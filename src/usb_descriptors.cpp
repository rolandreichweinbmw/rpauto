// usb_descriptors.cpp - USB descriptors for gs_usb/candleLight device
// Provides TinyUSB descriptor callbacks matching the candleLight USB layout
// so that the Linux gs_usb kernel driver recognizes this device.

#include "tusb.h"
#include "gs_usb.h"

// ============================================================================
// Device Descriptor - candleLight compatible
// ============================================================================
static const tusb_desc_device_t device_descriptor = {
    .bLength            = sizeof(tusb_desc_device_t),
    .bDescriptorType    = TUSB_DESC_DEVICE,
    .bcdUSB             = 0x0200,          // USB 2.0
    .bDeviceClass       = TUSB_CLASS_UNSPECIFIED,
    .bDeviceSubClass    = 0x00,
    .bDeviceProtocol    = 0x00,
    .bMaxPacketSize0    = CFG_TUD_ENDPOINT0_SIZE,
    .idVendor           = gs_usb::USB_VID, // 0x1209 (pid.codes)
    .idProduct          = gs_usb::USB_PID, // 0x2323 (candleLight)
    .bcdDevice          = 0x0100,          // Device version 1.0
    .iManufacturer      = 0x01,
    .iProduct           = 0x02,
    .iSerialNumber      = 0x03,
    .bNumConfigurations = 0x01,
};

// ============================================================================
// Configuration Descriptor
// Layout: Config -> Interface 0 (vendor, 2 bulk endpoints)
// ============================================================================
enum {
    ITF_NUM_GS_CAN = 0,
    ITF_NUM_TOTAL
};

// Total length: config (9) + interface (9) + endpoint IN (7) + endpoint OUT (7) = 32
#define CONFIG_TOTAL_LEN  (TUD_CONFIG_DESC_LEN + TUD_VENDOR_DESC_LEN)

static const uint8_t configuration_descriptor[] = {
    // Configuration descriptor
    TUD_CONFIG_DESCRIPTOR(
        1,                                    // bConfigurationValue
        ITF_NUM_TOTAL,                        // bNumInterfaces
        0,                                    // iConfiguration (string index)
        CONFIG_TOTAL_LEN,                     // wTotalLength
        0x80,                                 // bmAttributes: bus-powered
        150                                   // bMaxPower: 300mA
    ),

    // Vendor class interface (gs_usb)
    // Interface 0: class=0xFF (vendor), subclass=0xFF, protocol=0xFF
    // Bulk IN endpoint 0x81, Bulk OUT endpoint 0x02
    TUD_VENDOR_DESCRIPTOR(
        ITF_NUM_GS_CAN,                      // bInterfaceNumber
        0,                                    // iInterface (string index)
        gs_usb::ENDPOINT_OUT,                 // Bulk OUT endpoint
        gs_usb::ENDPOINT_IN,                  // Bulk IN endpoint
        gs_usb::CAN_DATA_MAX_PACKET_SIZE      // wMaxPacketSize
    ),
};

// ============================================================================
// String Descriptors
// ============================================================================
static const char* const string_descriptors[] = {
    "\x09\x04",         // 0: Language (English US)
    "Reichwein.IT,      // 1: Manufacturer
    "USB-CAN-FD",       // 2: Product
    "000000000001",     // 3: Serial Number
};

// ============================================================================
// TinyUSB descriptor callbacks
// ============================================================================
extern "C" {

const uint8_t* tud_descriptor_device_cb(void) {
    return reinterpret_cast<const uint8_t*>(&device_descriptor);
}

const uint8_t* tud_descriptor_configuration_cb(uint8_t index) {
    (void)index;
    return configuration_descriptor;
}

const uint16_t* tud_descriptor_string_cb(uint8_t index, uint16_t langid) {
    (void)langid;

    static uint16_t desc_str[32 + 1]; // UTF-16 buffer

    uint8_t chr_count;

    if (index == 0) {
        // Language descriptor
        desc_str[1] = 0x0409; // English US
        chr_count = 1;
    } else {
        if (index >= sizeof(string_descriptors) / sizeof(string_descriptors[0])) {
            return nullptr;
        }

        const char* str = string_descriptors[index];
        chr_count = 0;
        for (chr_count = 0; chr_count < 32 && str[chr_count]; chr_count++) {
            desc_str[1 + chr_count] = str[chr_count];
        }
    }

    // First word: length and type
    desc_str[0] = static_cast<uint16_t>((TUSB_DESC_STRING << 8) | (2 * chr_count + 2));

    return desc_str;
}

} // extern "C"
