// main.cpp - USB-CAN-FD adapter firmware for RP2350 + MCP2518FD
// Implements gs_usb protocol compatible with Linux gs_usb driver.

#include "pico/stdlib.h"
#include "tusb.h"
#include "gs_usb.h"
#include "usb_gs_can.h"
#include "mcp2518fd.h"
#include "mcp2518fd_regs.h"
#include "can_frame.h"
#include <cstring>

// ============================================================================
// Hardware Pin Definitions
// ============================================================================

// Status LED
static constexpr uint8_t PIN_LED = PICO_DEFAULT_LED_PIN;

// MCP2518FD SPI interface (directly directly directly directly using SPI0)
static constexpr uint8_t SPI_INST     = 0;      // spi0
static constexpr uint8_t PIN_SPI_MISO = 4;
static constexpr uint8_t PIN_SPI_MOSI = 3;
static constexpr uint8_t PIN_SPI_SCK  = 2;
static constexpr uint8_t PIN_SPI_CS   = 5;
static constexpr uint8_t PIN_CAN_INT  = 26;     // MCP2518FD INT pin

// MCP2518FD oscillator frequency (external crystal or internal)
static constexpr uint32_t MCP_OSC_FREQ_HZ = 40'000'000; // 40 MHz

// ============================================================================
// Global MCP2518FD Driver Instance
// ============================================================================

static mcp2518fd::PinConfig g_can_pins = {
    .spi_inst = SPI_INST,
    .pin_miso = PIN_SPI_MISO,
    .pin_mosi = PIN_SPI_MOSI,
    .pin_sck  = PIN_SPI_SCK,
    .pin_cs   = PIN_SPI_CS,
    .pin_int  = PIN_CAN_INT,
};

static mcp2518fd::Driver g_can_driver(g_can_pins);

// ============================================================================
// gs_usb Configuration (matching MCP2518FD capabilities)
// ============================================================================

// Features supported by this device
static constexpr uint32_t GS_FEATURES =
    gs_usb::FEATURE_LISTEN_ONLY |
    gs_usb::FEATURE_LOOP_BACK |
    gs_usb::FEATURE_HW_TIMESTAMP |
    gs_usb::FEATURE_IDENTIFY |
    gs_usb::FEATURE_FD |
    gs_usb::FEATURE_BT_CONST_EXT |
    gs_usb::FEATURE_GET_STATE;

// Nominal bit timing constants (from MCP2518FD datasheet)
// CiNBTCFG: BRP[7:0], TSEG1[7:0], TSEG2[6:0], SJW[6:0]
static constexpr gs_usb::bittiming_const GS_BT_CONST = {
    .tseg1_min = 2,
    .tseg1_max = 256,
    .tseg2_min = 1,
    .tseg2_max = 128,
    .sjw_max   = 128,
    .brp_min   = 1,
    .brp_max   = 256,
    .brp_inc   = 1,
};

// Data bit timing constants (CAN-FD data phase)
// CiDBTCFG: BRP[7:0], TSEG1[4:0], TSEG2[3:0], SJW[3:0]
static constexpr gs_usb::bittiming_const GS_DBT_CONST = {
    .tseg1_min = 1,
    .tseg1_max = 32,
    .tseg2_min = 1,
    .tseg2_max = 16,
    .sjw_max   = 16,
    .brp_min   = 1,
    .brp_max   = 256,
    .brp_inc   = 1,
};

// ============================================================================
// Callbacks from USB host commands
// ============================================================================

// Called when host sets nominal bit timing
static void on_bittiming(uint8_t channel, const gs_usb::device_bittiming& bt) {
    (void)channel;

    // gs_usb bittiming uses prop_seg + phase_seg1 for TSEG1
    // MCP2518FD register values are 0-based (actual = reg + 1)
    mcp2518fd::BitTimingConfig cfg = {
        .brp   = static_cast<uint8_t>(bt.brp - 1),
        .tseg1 = static_cast<uint8_t>(bt.prop_seg + bt.phase_seg1 - 1),
        .tseg2 = static_cast<uint8_t>(bt.phase_seg2 - 1),
        .sjw   = static_cast<uint8_t>(bt.sjw - 1),
    };
    g_can_driver.set_nominal_bittiming(cfg);
}

// Called when host sets data phase bit timing (CAN-FD)
static void on_data_bittiming(uint8_t channel, const gs_usb::device_bittiming& bt) {
    (void)channel;

    mcp2518fd::BitTimingConfig cfg = {
        .brp   = static_cast<uint8_t>(bt.brp - 1),
        .tseg1 = static_cast<uint8_t>(bt.prop_seg + bt.phase_seg1 - 1),
        .tseg2 = static_cast<uint8_t>(bt.phase_seg2 - 1),
        .sjw   = static_cast<uint8_t>(bt.sjw - 1),
    };
    g_can_driver.set_data_bittiming(cfg);
}

// Called when host starts or resets a CAN channel
static void on_mode(uint8_t channel, gs_usb::can_mode mode, uint32_t flags) {
    (void)channel;

    if (mode == gs_usb::can_mode::START) {
        // Determine MCP2518FD mode from gs_usb flags
        uint8_t mcp_mode = mcp2518fd::CAN_MODE_NORMAL_FD;

        if (flags & gs_usb::MODE_LISTEN_ONLY) {
            mcp_mode = mcp2518fd::CAN_MODE_LISTEN_ONLY;
        } else if (flags & gs_usb::MODE_LOOP_BACK) {
            mcp_mode = mcp2518fd::CAN_MODE_INT_LOOPBACK;
        } else if (!(flags & gs_usb::MODE_FD)) {
            // If FD not requested, use CAN 2.0 mode
            mcp_mode = mcp2518fd::CAN_MODE_NORMAL_CAN20;
        }

        g_can_driver.set_mode(mcp_mode);
        gpio_put(PIN_LED, true); // LED on when channel active
    } else {
        // RESET: go back to configuration mode
        g_can_driver.set_mode(mcp2518fd::CAN_MODE_CONFIG);
        gpio_put(PIN_LED, false);
    }
}

// Called when host requests identify (LED blink)
static volatile bool g_identify_active = false;

static void on_identify(uint8_t channel, bool on) {
    (void)channel;
    g_identify_active = on;
}

// ============================================================================
// Frame conversion helpers
// ============================================================================

// Convert a received CAN-FD frame to gs_usb host_frame format
static size_t can_to_gs_frame(const can::FdFrame& can_frame,
                               gs_usb::host_frame& gs_frame,
                               uint8_t channel,
                               bool use_timestamp) {
    gs_frame.echo_id  = gs_usb::HOST_FRAME_ECHO_ID_RX; // Received frame
    gs_frame.can_id   = can_frame.id;
    gs_frame.can_dlc  = can_frame.dlc;
    gs_frame.channel  = channel;
    gs_frame.reserved = 0;

    // Set flags
    gs_frame.flags = 0;
    if (can_frame.dlc > 8 || can_frame.is_brs()) {
        gs_frame.flags |= gs_usb::FLAG_FD;
    }
    if (can_frame.is_brs()) {
        gs_frame.flags |= gs_usb::FLAG_BRS;
    }
    if (can_frame.is_esi()) {
        gs_frame.flags |= gs_usb::FLAG_ESI;
    }

    // Copy data
    uint8_t data_len = can_frame.length();
    std::memset(&gs_frame.canfd_ts, 0, sizeof(gs_frame.canfd_ts));
    std::memcpy(gs_frame.canfd_ts.data, can_frame.data.data(), data_len);

    // Add timestamp if enabled
    if (use_timestamp) {
        gs_frame.canfd_ts.timestamp_us = usb_gs_can::get_timestamp_us();
        if (gs_frame.flags & gs_usb::FLAG_FD) {
            return gs_usb::HOST_FRAME_SIZE_CANFD_TS;
        }
        return gs_usb::HOST_FRAME_SIZE_CLASSIC_TS;
    }

    if (gs_frame.flags & gs_usb::FLAG_FD) {
        return gs_usb::HOST_FRAME_SIZE_CANFD;
    }
    return gs_usb::HOST_FRAME_SIZE_CLASSIC;
}

// Convert a gs_usb host_frame from host to CAN-FD frame for transmission
static void gs_to_can_frame(const gs_usb::host_frame& gs_frame,
                            can::FdFrame& can_frame) {
    can_frame.id = gs_frame.can_id;
    can_frame.dlc = gs_frame.can_dlc;
    can_frame.fd_flags = 0;

    if (gs_frame.flags & gs_usb::FLAG_BRS) {
        can_frame.fd_flags |= can::FD_FLAG_BRS;
    }

    // Copy data
    uint8_t data_len = can_frame.length();
    can_frame.data.fill(0);
    if (gs_frame.flags & gs_usb::FLAG_FD) {
        std::memcpy(can_frame.data.data(), gs_frame.canfd.data, data_len);
    } else {
        std::memcpy(can_frame.data.data(), gs_frame.classic_can.data,
                    (data_len > 8) ? 8 : data_len);
    }
}

// ============================================================================
// Main Entry Point
// ============================================================================

int main() {
    // --- Basic initialization ---
    stdio_init_all();

    // LED for status
    gpio_init(PIN_LED);
    gpio_set_dir(PIN_LED, GPIO_OUT);
    gpio_put(PIN_LED, false);

    // --- Initialize TinyUSB ---
    tusb_init();

    // --- Initialize MCP2518FD ---
    if (!g_can_driver.init()) {
        // Initialization failed — blink LED rapidly to indicate error
        while (true) {
            gpio_put(PIN_LED, true);
            sleep_ms(100);
            gpio_put(PIN_LED, false);
            sleep_ms(100);
        }
    }

    // --- Configure gs_usb layer ---
    usb_gs_can::Config gs_config = {
        .channel_count = 1,
        .sw_version    = 2,      // Firmware version
        .hw_version    = 1,      // Hardware version
        .can_clock_hz  = MCP_OSC_FREQ_HZ,
        .features      = GS_FEATURES,
        .bt_const      = GS_BT_CONST,
        .dbt_const     = GS_DBT_CONST,
    };
    usb_gs_can::init(gs_config);

    // --- Register callbacks ---
    usb_gs_can::set_bittiming_callback(
        usb_gs_can::bittiming_cb_t::create<on_bittiming>());
    usb_gs_can::set_data_bittiming_callback(
        usb_gs_can::data_bittiming_cb_t::create<on_data_bittiming>());
    usb_gs_can::set_mode_callback(
        usb_gs_can::mode_cb_t::create<on_mode>());
    usb_gs_can::set_identify_callback(
        usb_gs_can::identify_cb_t::create<on_identify>());

    // --- Variables for identify LED blinking ---
    uint32_t identify_toggle_time = 0;
    bool identify_led_state = false;

    // --- Main loop ---
    while (true) {
        // Run TinyUSB device task
        tud_task();

        // Run gs_usb task (processes incoming USB frames)
        usb_gs_can::task();

        // --- Bridge: USB -> CAN (TX) ---
        if (usb_gs_can::frame_from_host_available()) {
            gs_usb::host_frame gs_frame;
            size_t len = usb_gs_can::receive_frame_from_host(gs_frame);
            if (len > 0) {
                can::FdFrame can_frame;
                gs_to_can_frame(gs_frame, can_frame);

                if (g_can_driver.transmit(can_frame)) {
                    // TX successful — send echo back to host
                    // (echo_id is preserved from the host frame)
                    gs_frame.echo_id = gs_frame.echo_id; // already set by host
                    size_t echo_size = (gs_frame.flags & gs_usb::FLAG_FD)
                        ? gs_usb::HOST_FRAME_SIZE_CANFD
                        : gs_usb::HOST_FRAME_SIZE_CLASSIC;
                    usb_gs_can::send_frame_to_host(gs_frame, echo_size);
                }
            }
        }

        // --- Bridge: CAN -> USB (RX) ---
        while (g_can_driver.rx_available()) {
            can::FdFrame can_frame;
            if (g_can_driver.receive(can_frame)) {
                gs_usb::host_frame gs_frame;
                size_t frame_size = can_to_gs_frame(can_frame, gs_frame, 0, true);
                usb_gs_can::send_frame_to_host(gs_frame, frame_size);
            }
        }

        // --- Identify LED blinking ---
        if (g_identify_active) {
            uint32_t now = to_ms_since_boot(get_absolute_time());
            if (now - identify_toggle_time >= 500) {
                identify_toggle_time = now;
                identify_led_state = !identify_led_state;
                gpio_put(PIN_LED, identify_led_state);
            }
        }
    }

    return 0;
}
