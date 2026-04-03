// gs_usb.h - gs_usb/candleLight protocol definitions
// Matches the Linux kernel gs_usb driver expectations.
// All data exchanged in little-endian byte order.
#pragma once

#include <cstdint>
#include <cstddef>

namespace gs_usb {

// --- Endpoint addresses (matching candleLight firmware) ---
static constexpr uint8_t ENDPOINT_IN  = 0x81;
static constexpr uint8_t ENDPOINT_OUT = 0x02;

// --- USB IDs (candleLight) ---
static constexpr uint16_t USB_VID = 0x1209;
static constexpr uint16_t USB_PID = 0x2323;

// --- Vendor requests (bRequest values for control transfers) ---
enum class breq : uint8_t {
    HOST_FORMAT       = 0,
    BITTIMING         = 1,
    MODE              = 2,
    BERR              = 3,
    BT_CONST          = 4,
    DEVICE_CONFIG     = 5,
    TIMESTAMP         = 6,
    IDENTIFY          = 7,
    GET_USER_ID       = 8,
    SET_USER_ID       = 9,
    DATA_BITTIMING    = 10,
    BT_CONST_EXT      = 11,
    SET_TERMINATION   = 12,
    GET_TERMINATION   = 13,
    GET_STATE         = 14,
};

// --- Feature flags (reported in bt_const.feature) ---
static constexpr uint32_t FEATURE_LISTEN_ONLY              = (1u << 0);
static constexpr uint32_t FEATURE_LOOP_BACK                = (1u << 1);
static constexpr uint32_t FEATURE_TRIPLE_SAMPLE            = (1u << 2);
static constexpr uint32_t FEATURE_ONE_SHOT                 = (1u << 3);
static constexpr uint32_t FEATURE_HW_TIMESTAMP             = (1u << 4);
static constexpr uint32_t FEATURE_IDENTIFY                 = (1u << 5);
static constexpr uint32_t FEATURE_USER_ID                  = (1u << 6);
static constexpr uint32_t FEATURE_PAD_PKTS_TO_MAX_PKT_SIZE = (1u << 7);
static constexpr uint32_t FEATURE_FD                       = (1u << 8);
static constexpr uint32_t FEATURE_REQ_USB_QUIRK_LPC546XX   = (1u << 9);
static constexpr uint32_t FEATURE_BT_CONST_EXT             = (1u << 10);
static constexpr uint32_t FEATURE_TERMINATION              = (1u << 11);
static constexpr uint32_t FEATURE_BERR_REPORTING           = (1u << 12);
static constexpr uint32_t FEATURE_GET_STATE                = (1u << 13);

// --- CAN frame flags (in host_frame.flags) ---
static constexpr uint8_t FLAG_OVERFLOW = (1u << 0);
static constexpr uint8_t FLAG_FD       = (1u << 1);
static constexpr uint8_t FLAG_BRS      = (1u << 2);
static constexpr uint8_t FLAG_ESI      = (1u << 3);

// --- CAN mode flags (in device_mode.flags) ---
static constexpr uint32_t MODE_NORMAL                    = 0;
static constexpr uint32_t MODE_LISTEN_ONLY               = (1u << 0);
static constexpr uint32_t MODE_LOOP_BACK                 = (1u << 1);
static constexpr uint32_t MODE_TRIPLE_SAMPLE             = (1u << 2);
static constexpr uint32_t MODE_ONE_SHOT                  = (1u << 3);
static constexpr uint32_t MODE_HW_TIMESTAMP              = (1u << 4);
static constexpr uint32_t MODE_PAD_PKTS_TO_MAX_PKT_SIZE  = (1u << 7);
static constexpr uint32_t MODE_FD                        = (1u << 8);
static constexpr uint32_t MODE_BERR_REPORTING            = (1u << 12);

// --- CAN mode (mode field) ---
enum class can_mode : uint32_t {
    RESET = 0,
    START = 1,
};

// --- CAN state ---
enum class can_state : uint32_t {
    ERROR_ACTIVE  = 0,
    ERROR_WARNING = 1,
    ERROR_PASSIVE = 2,
    BUS_OFF       = 3,
    STOPPED       = 4,
    SLEEPING      = 5,
};

// --- CAN ID flags (matching Linux CAN definitions) ---
static constexpr uint32_t CAN_EFF_FLAG = 0x80000000u;
static constexpr uint32_t CAN_RTR_FLAG = 0x40000000u;
static constexpr uint32_t CAN_ERR_FLAG = 0x20000000u;

// --- Echo ID indicating a received (not echoed) frame ---
static constexpr uint32_t HOST_FRAME_ECHO_ID_RX = 0xFFFFFFFFu;

// --- Protocol structs (all little-endian, packed) ---

struct host_config {
    uint32_t byte_order;
} __attribute__((packed));

struct device_config {
    uint8_t  reserved1;
    uint8_t  reserved2;
    uint8_t  reserved3;
    uint8_t  icount;       // number of CAN channels - 1
    uint32_t sw_version;
    uint32_t hw_version;
} __attribute__((packed));

struct device_mode {
    uint32_t mode;   // can_mode (RESET or START)
    uint32_t flags;  // combination of MODE_* flags
} __attribute__((packed));

struct device_state {
    uint32_t state;  // can_state
    uint32_t rxerr;
    uint32_t txerr;
} __attribute__((packed));

struct device_bittiming {
    uint32_t prop_seg;
    uint32_t phase_seg1;
    uint32_t phase_seg2;
    uint32_t sjw;
    uint32_t brp;
} __attribute__((packed));

struct bittiming_const {
    uint32_t tseg1_min;
    uint32_t tseg1_max;
    uint32_t tseg2_min;
    uint32_t tseg2_max;
    uint32_t sjw_max;
    uint32_t brp_min;
    uint32_t brp_max;
    uint32_t brp_inc;
} __attribute__((packed));

struct device_bt_const {
    uint32_t         feature;
    uint32_t         fclk_can;
    bittiming_const  btc;
} __attribute__((packed));

struct device_bt_const_extended {
    uint32_t         feature;
    uint32_t         fclk_can;
    bittiming_const  btc;
    bittiming_const  dbtc;   // data phase bit timing
} __attribute__((packed));

struct identify_mode {
    uint32_t mode;   // 0 = off, 1 = on
} __attribute__((packed));

struct device_termination_state {
    uint32_t state;  // 0 = off, 1 = on
} __attribute__((packed));

// --- CAN data payload variants ---

struct classic_can_data {
    uint8_t data[8];
} __attribute__((packed));

struct classic_can_data_ts {
    uint8_t  data[8];
    uint32_t timestamp_us;
} __attribute__((packed));

struct canfd_data {
    uint8_t data[64];
} __attribute__((packed));

struct canfd_data_ts {
    uint8_t  data[64];
    uint32_t timestamp_us;
} __attribute__((packed));

// --- Host frame header (common part) ---
struct host_frame_header {
    uint32_t echo_id;
    uint32_t can_id;
    uint8_t  can_dlc;
    uint8_t  channel;
    uint8_t  flags;
    uint8_t  reserved;
} __attribute__((packed));

// --- Host frame: classic CAN with timestamp (most common for our FD device) ---
struct host_frame {
    uint32_t echo_id;
    uint32_t can_id;
    uint8_t  can_dlc;
    uint8_t  channel;
    uint8_t  flags;
    uint8_t  reserved;
    union {
        classic_can_data    classic_can;
        classic_can_data_ts classic_can_ts;
        canfd_data          canfd;
        canfd_data_ts       canfd_ts;
    };
} __attribute__((packed));

// --- Frame size helpers ---
static constexpr size_t HOST_FRAME_SIZE_CLASSIC    = sizeof(host_frame_header) + sizeof(classic_can_data);
static constexpr size_t HOST_FRAME_SIZE_CLASSIC_TS = sizeof(host_frame_header) + sizeof(classic_can_data_ts);
static constexpr size_t HOST_FRAME_SIZE_CANFD      = sizeof(host_frame_header) + sizeof(canfd_data);
static constexpr size_t HOST_FRAME_SIZE_CANFD_TS   = sizeof(host_frame_header) + sizeof(canfd_data_ts);
static constexpr size_t HOST_FRAME_MAX_SIZE        = sizeof(host_frame);

// Maximum USB packet size for CAN data endpoint (must fit largest frame)
static constexpr uint16_t CAN_DATA_MAX_PACKET_SIZE = 128;

} // namespace gs_usb
