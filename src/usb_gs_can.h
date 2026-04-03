// usb_gs_can.h - USB gs_usb vendor device class interface
// Bridges TinyUSB vendor class to gs_usb protocol handling.
#pragma once

#include <cstdint>
#include "gs_usb.h"
#include "can_frame.h"
#include <etl/delegate.h>

namespace usb_gs_can {

// --- Callbacks from USB events to application ---

// Called when the host sets bit timing (nominal)
using bittiming_cb_t = etl::delegate<void(uint8_t channel, const gs_usb::device_bittiming&)>;

// Called when the host sets data bit timing (CAN-FD)
using data_bittiming_cb_t = etl::delegate<void(uint8_t channel, const gs_usb::device_bittiming&)>;

// Called when the host starts or resets a CAN channel
using mode_cb_t = etl::delegate<void(uint8_t channel, gs_usb::can_mode mode, uint32_t flags)>;

// Called when the host sends an identify request
using identify_cb_t = etl::delegate<void(uint8_t channel, bool on)>;

// --- Configuration ---
struct Config {
    uint8_t  channel_count;   // number of CAN channels (typically 1)
    uint32_t sw_version;
    uint32_t hw_version;
    uint32_t can_clock_hz;    // CAN peripheral clock (e.g. 40 MHz for MCP2518FD)

    // Feature flags advertised to the host
    uint32_t features;

    // Bit timing constants (nominal)
    gs_usb::bittiming_const bt_const;

    // Bit timing constants (data, for CAN-FD)
    gs_usb::bittiming_const dbt_const;
};

// --- Initialization ---
void init(const Config& config);

// --- Register callbacks ---
void set_bittiming_callback(bittiming_cb_t cb);
void set_data_bittiming_callback(data_bittiming_cb_t cb);
void set_mode_callback(mode_cb_t cb);
void set_identify_callback(identify_cb_t cb);

// --- Main loop task (call from main loop, processes USB events) ---
void task();

// --- Send a received CAN frame to the host (bulk IN) ---
// Returns true if the frame was queued successfully.
bool send_frame_to_host(const gs_usb::host_frame& frame, size_t frame_size);

// --- Check if a frame from host is pending ---
bool frame_from_host_available();

// --- Get a CAN frame sent by the host (bulk OUT) ---
// Returns the number of bytes received, or 0 if none available.
size_t receive_frame_from_host(gs_usb::host_frame& frame);

// --- Get current timestamp in microseconds ---
uint32_t get_timestamp_us();

} // namespace usb_gs_can
