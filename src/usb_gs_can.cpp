// usb_gs_can.cpp - USB gs_usb vendor device class implementation
// Handles vendor control requests and bulk CAN frame transfers via TinyUSB.

#include "usb_gs_can.h"
#include "tusb.h"
#include "gs_usb.h"
#include "pico/time.h"
#include <cstring>

// Maximum number of CAN channels supported
static constexpr uint8_t MAX_CHANNELS = 1;

namespace usb_gs_can {

// ============================================================================
// Part A: Internal state
// ============================================================================

// Stored configuration (set once at init)
static Config s_config{};

// Pre-built response structs (populated at init, sent to host on request)
static gs_usb::device_config     s_device_config{};
static gs_usb::device_bt_const   s_bt_const{};
static gs_usb::device_bt_const_extended s_bt_const_ext{};

// Per-channel state
struct ChannelState {
    bool     active{false};
    uint32_t mode_flags{0};
};
static ChannelState s_channels[MAX_CHANNELS]{};

// Timestamp (microseconds, from RP2350 timer)
static uint32_t s_timestamp_us{0};

// EP0 receive buffer for control OUT data (host -> device)
static union {
    uint8_t                         raw[64];
    gs_usb::host_config             host_config;
    gs_usb::device_bittiming        bittiming;
    gs_usb::device_mode             mode;
    gs_usb::identify_mode           identify;
    gs_usb::device_termination_state termination;
} s_ep0_buf{};

// Bulk OUT receive buffer
static gs_usb::host_frame s_rx_frame{};
static volatile bool      s_rx_frame_available{false};

// Bulk IN: tracks whether a transmission is in progress
static volatile bool s_tx_busy{false};

// Registered callbacks
static bittiming_cb_t      s_bittiming_cb{};
static data_bittiming_cb_t s_data_bittiming_cb{};
static mode_cb_t           s_mode_cb{};
static identify_cb_t       s_identify_cb{};

// The last control SETUP request, needed to know which breq the DATA stage belongs to
static uint8_t s_last_breq{0xFF};
static uint16_t s_last_wvalue{0};

void init(const Config& config) {
    s_config = config;

    // Build device_config response
    s_device_config.reserved1  = 0;
    s_device_config.reserved2  = 0;
    s_device_config.reserved3  = 0;
    s_device_config.icount     = config.channel_count - 1; // gs_usb uses 0-based count
    s_device_config.sw_version = config.sw_version;
    s_device_config.hw_version = config.hw_version;

    // Build bt_const response (for BREQ_BT_CONST)
    s_bt_const.feature  = config.features;
    s_bt_const.fclk_can = config.can_clock_hz;
    s_bt_const.btc      = config.bt_const;

    // Build extended bt_const response (for BREQ_BT_CONST_EXT)
    s_bt_const_ext.feature  = config.features;
    s_bt_const_ext.fclk_can = config.can_clock_hz;
    s_bt_const_ext.btc      = config.bt_const;
    s_bt_const_ext.dbtc     = config.dbt_const;

    // Reset channel state
    for (auto& ch : s_channels) {
        ch.active     = false;
        ch.mode_flags = 0;
    }

    s_rx_frame_available = false;
    s_tx_busy            = false;
    s_last_breq          = 0xFF;
}

uint32_t get_timestamp_us() {
    s_timestamp_us = static_cast<uint32_t>(time_us_64());
    return s_timestamp_us;
}

void task() {
    // If no pending frame and vendor endpoint has data, read it
    if (!s_rx_frame_available && tud_vendor_available()) {
        uint32_t count = tud_vendor_read(&s_rx_frame, sizeof(s_rx_frame));
        if (count >= gs_usb::HOST_FRAME_SIZE_CLASSIC) {
            s_rx_frame_available = true;
        }
    }
}

bool send_frame_to_host(const gs_usb::host_frame& frame, size_t frame_size) {
    if (!tud_vendor_mounted()) {
        return false;
    }
    // Write the frame to the bulk IN endpoint
    uint32_t written = tud_vendor_write(reinterpret_cast<const uint8_t*>(&frame),
                                         static_cast<uint32_t>(frame_size));
    if (written > 0) {
        tud_vendor_flush();
        return true;
    }
    return false;
}

bool frame_from_host_available() {
    return s_rx_frame_available;
}

size_t receive_frame_from_host(gs_usb::host_frame& frame) {
    if (!s_rx_frame_available) {
        return 0;
    }
    std::memcpy(&frame, &s_rx_frame, sizeof(gs_usb::host_frame));
    s_rx_frame_available = false;

    // Determine actual size based on flags
    if (frame.flags & gs_usb::FLAG_FD) {
        if (s_channels[0].mode_flags & gs_usb::MODE_HW_TIMESTAMP) {
            return gs_usb::HOST_FRAME_SIZE_CANFD_TS;
        }
        return gs_usb::HOST_FRAME_SIZE_CANFD;
    }
    if (s_channels[0].mode_flags & gs_usb::MODE_HW_TIMESTAMP) {
        return gs_usb::HOST_FRAME_SIZE_CLASSIC_TS;
    }
    return gs_usb::HOST_FRAME_SIZE_CLASSIC;
}

void set_bittiming_callback(bittiming_cb_t cb) {
    s_bittiming_cb = cb;
}

void set_data_bittiming_callback(data_bittiming_cb_t cb) {
    s_data_bittiming_cb = cb;
}

void set_mode_callback(mode_cb_t cb) {
    s_mode_cb = cb;
}

void set_identify_callback(identify_cb_t cb) {
    s_identify_cb = cb;
}

} // namespace usb_gs_can

// ============================================================================
// TinyUSB vendor class callbacks
// ============================================================================

extern "C" {

// ---------------------------------------------------------------------------
// Helper: handle SETUP stage — decide what to send/receive for each breq
// ---------------------------------------------------------------------------
static bool handle_setup_stage(uint8_t rhport, tusb_control_request_t const* request) {
    using namespace usb_gs_can;
    using namespace gs_usb;

    const uint8_t  breq_val = request->bRequest;
    const uint16_t wValue   = request->wValue;
    //const uint16_t wLength  = request->wLength;
    //const bool     dir_in   = (request->bmRequestType_bit.direction == TUSB_DIR_IN);

    // Remember for DATA stage
    s_last_breq   = breq_val;
    s_last_wvalue = wValue;

    switch (static_cast<breq>(breq_val)) {

    // ---- Host -> Device: host sends its byte order (we ignore it, always LE) ----
    case breq::HOST_FORMAT:
        return tud_control_xfer(rhport, request, s_ep0_buf.raw,
                                sizeof(gs_usb::host_config));

    // ---- Device -> Host: send device configuration ----
    case breq::DEVICE_CONFIG:
        return tud_control_xfer(rhport, request,
                                reinterpret_cast<void*>(&s_device_config),
                                sizeof(s_device_config));

    // ---- Device -> Host: send bit timing constants ----
    case breq::BT_CONST:
        return tud_control_xfer(rhport, request,
                                reinterpret_cast<void*>(&s_bt_const),
                                sizeof(s_bt_const));

    // ---- Device -> Host: send extended bit timing constants (CAN-FD) ----
    case breq::BT_CONST_EXT:
        return tud_control_xfer(rhport, request,
                                reinterpret_cast<void*>(&s_bt_const_ext),
                                sizeof(s_bt_const_ext));

    // ---- Host -> Device: set nominal bit timing ----
    case breq::BITTIMING:
        return tud_control_xfer(rhport, request, s_ep0_buf.raw,
                                sizeof(gs_usb::device_bittiming));

    // ---- Host -> Device: set data bit timing (CAN-FD) ----
    case breq::DATA_BITTIMING:
        return tud_control_xfer(rhport, request, s_ep0_buf.raw,
                                sizeof(gs_usb::device_bittiming));

    // ---- Host -> Device: set mode (start/reset channel) ----
    case breq::MODE:
        return tud_control_xfer(rhport, request, s_ep0_buf.raw,
                                sizeof(gs_usb::device_mode));

    // ---- Device -> Host: send current timestamp ----
    case breq::TIMESTAMP: {
        s_timestamp_us = static_cast<uint32_t>(time_us_64());
        return tud_control_xfer(rhport, request,
                                reinterpret_cast<void*>(&s_timestamp_us),
                                sizeof(s_timestamp_us));
    }

    // ---- Host -> Device: identify (LED blink) ----
    case breq::IDENTIFY:
        return tud_control_xfer(rhport, request, s_ep0_buf.raw,
                                sizeof(gs_usb::identify_mode));

    // ---- Device -> Host: get state (error counters) ----
    case breq::GET_STATE: {
        static gs_usb::device_state dev_state{};
        dev_state.state = static_cast<uint32_t>(can_state::ERROR_ACTIVE);
        dev_state.rxerr = 0;
        dev_state.txerr = 0;
        return tud_control_xfer(rhport, request,
                                reinterpret_cast<void*>(&dev_state),
                                sizeof(dev_state));
    }

    // ---- Device -> Host: get termination state ----
    case breq::GET_TERMINATION: {
        static gs_usb::device_termination_state term{};
        term.state = 0; // off
        return tud_control_xfer(rhport, request,
                                reinterpret_cast<void*>(&term),
                                sizeof(term));
    }

    // ---- Host -> Device: set termination (we acknowledge but ignore) ----
    case breq::SET_TERMINATION:
        return tud_control_xfer(rhport, request, s_ep0_buf.raw,
                                sizeof(gs_usb::device_termination_state));

    default:
        // Unknown request — stall
        return false;
    }
}

// ---------------------------------------------------------------------------
// Helper: handle DATA stage — process data received from host
// ---------------------------------------------------------------------------
static bool handle_data_stage(uint8_t rhport, tusb_control_request_t const* request) {
    using namespace usb_gs_can;
    using namespace gs_usb;

    const uint8_t  breq_val = s_last_breq;
    const uint16_t channel  = s_last_wvalue;

    switch (static_cast<breq>(breq_val)) {

    case breq::HOST_FORMAT:
        // candleLight ignores byte_order, always uses LE. Nothing to do.
        return true;

    case breq::BITTIMING:
        if (s_bittiming_cb.is_valid() && channel < MAX_CHANNELS) {
            s_bittiming_cb(static_cast<uint8_t>(channel), s_ep0_buf.bittiming);
        }
        return true;

    case breq::DATA_BITTIMING:
        if (s_data_bittiming_cb.is_valid() && channel < MAX_CHANNELS) {
            s_data_bittiming_cb(static_cast<uint8_t>(channel), s_ep0_buf.bittiming);
        }
        return true;

    case breq::MODE:
        if (channel < MAX_CHANNELS) {
            auto& ch = s_channels[channel];
            auto mode = static_cast<can_mode>(s_ep0_buf.mode.mode);
            uint32_t flags = s_ep0_buf.mode.flags;

            if (mode == can_mode::START) {
                ch.active     = true;
                ch.mode_flags = flags;
            } else {
                ch.active     = false;
                ch.mode_flags = 0;
            }

            if (s_mode_cb.is_valid()) {
                s_mode_cb(static_cast<uint8_t>(channel), mode, flags);
            }
        }
        return true;

    case breq::IDENTIFY:
        if (s_identify_cb.is_valid() && channel < MAX_CHANNELS) {
            s_identify_cb(static_cast<uint8_t>(channel),
                          s_ep0_buf.identify.mode != 0);
        }
        return true;

    case breq::SET_TERMINATION:
        // Acknowledge but no action (no HW termination support)
        return true;

    default:
        return true;
    }
}

// ---------------------------------------------------------------------------
// TinyUSB vendor control transfer callback (called for each stage)
// ---------------------------------------------------------------------------
bool tud_vendor_control_xfer_cb(uint8_t rhport, uint8_t stage,
                                tusb_control_request_t const* request) {
    // Only handle vendor requests to interface
    if (request->bmRequestType_bit.type != TUSB_REQ_TYPE_VENDOR) {
        return false;
    }

    if (stage == CONTROL_STAGE_SETUP) {
        return handle_setup_stage(rhport, request);
    } else if (stage == CONTROL_STAGE_DATA) {
        return handle_data_stage(rhport, request);
    }

    // ACK stage — nothing to do
    return true;
}

} // extern "C"
