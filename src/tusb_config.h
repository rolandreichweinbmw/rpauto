// tusb_config.h - TinyUSB configuration for RP2350 gs_usb device
#pragma once

// --- Board / MCU ---
#define CFG_TUSB_MCU          OPT_MCU_RP2040  // RP2350 uses same TinyUSB MCU option
#define CFG_TUSB_OS           OPT_OS_PICO

// --- RHPort mode configuration ---
// Use RHPort0 in device mode, full-speed
#define CFG_TUSB_RHPORT0_MODE (OPT_MODE_DEVICE | OPT_MODE_FULL_SPEED)

// --- USB device configuration ---
#define CFG_TUD_ENABLED       1
#define CFG_TUD_MAX_SPEED     OPT_MODE_FULL_SPEED

// --- Endpoint 0 size ---
#define CFG_TUD_ENDPOINT0_SIZE  64

// --- Class driver selection ---
// We use the vendor class for gs_usb protocol
#define CFG_TUD_VENDOR        1
#define CFG_TUD_CDC           0
#define CFG_TUD_MSC           0
#define CFG_TUD_HID           0
#define CFG_TUD_MIDI          0

// --- Vendor class configuration ---
// RX and TX buffer sizes for bulk endpoints
// Must be >= max CAN-FD frame size (header 12 + data 64 + timestamp 4 = 80)
// Use 128 to allow some headroom
#define CFG_TUD_VENDOR_RX_BUFSIZE  128
#define CFG_TUD_VENDOR_TX_BUFSIZE  128
#define CFG_TUD_VENDOR_EP_BUFSIZE  128

// --- Memory alignment ---
#define CFG_TUSB_MEM_ALIGN    __attribute__((aligned(4)))
