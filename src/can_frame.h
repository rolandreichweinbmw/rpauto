// can_frame.h - CAN / CAN-FD frame types using ETL
#pragma once

#include <cstdint>
#include <cstddef>
#include <etl/array.h>

namespace can {

// --- CAN ID flags ---
static constexpr uint32_t ID_EFF_FLAG  = 0x80000000u; // Extended frame format
static constexpr uint32_t ID_RTR_FLAG  = 0x40000000u; // Remote transmission request
static constexpr uint32_t ID_ERR_FLAG  = 0x20000000u; // Error message frame
static constexpr uint32_t ID_EFF_MASK  = 0x1FFFFFFFu; // 29-bit extended ID mask
static constexpr uint32_t ID_SFF_MASK  = 0x000007FFu; // 11-bit standard ID mask

// --- CAN-FD frame flags ---
static constexpr uint8_t FD_FLAG_BRS = 0x01; // Bit rate switch
static constexpr uint8_t FD_FLAG_ESI = 0x02; // Error state indicator

// --- Maximum data sizes ---
static constexpr size_t CLASSIC_MAX_DATA = 8;
static constexpr size_t FD_MAX_DATA      = 64;

// --- DLC to data length lookup table ---
inline constexpr uint8_t DLC_TO_LEN_TABLE[16] = {
    0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64
};

// --- DLC to data length conversion ---
inline constexpr uint8_t dlc_to_len(uint8_t dlc) {
    return (dlc < 16) ? DLC_TO_LEN_TABLE[dlc] : 64;
}

// --- Data length to DLC conversion ---
inline constexpr uint8_t len_to_dlc(uint8_t len) {
    if (len <= 8)  return len;
    if (len <= 12) return 9;
    if (len <= 16) return 10;
    if (len <= 20) return 11;
    if (len <= 24) return 12;
    if (len <= 32) return 13;
    if (len <= 48) return 14;
    return 15;
}

// --- Classic CAN frame ---
struct Frame {
    uint32_t                        id{0};      // CAN ID with flags
    uint8_t                         dlc{0};     // Data length code (0-8)
    etl::array<uint8_t, CLASSIC_MAX_DATA> data{};

    bool is_extended() const { return (id & ID_EFF_FLAG) != 0; }
    bool is_rtr()      const { return (id & ID_RTR_FLAG) != 0; }
    bool is_error()    const { return (id & ID_ERR_FLAG) != 0; }
    uint32_t raw_id()  const { return is_extended() ? (id & ID_EFF_MASK) : (id & ID_SFF_MASK); }
    uint8_t  length()  const { return (dlc <= 8) ? dlc : 8; }
};

// --- CAN-FD frame ---
struct FdFrame {
    uint32_t                     id{0};       // CAN ID with flags
    uint8_t                      dlc{0};      // DLC (0-15, maps to 0-64 bytes)
    uint8_t                      fd_flags{0}; // FD_FLAG_BRS, FD_FLAG_ESI
    etl::array<uint8_t, FD_MAX_DATA> data{};

    bool is_extended() const { return (id & ID_EFF_FLAG) != 0; }
    bool is_rtr()      const { return (id & ID_RTR_FLAG) != 0; }
    bool is_error()    const { return (id & ID_ERR_FLAG) != 0; }
    bool is_brs()      const { return (fd_flags & FD_FLAG_BRS) != 0; }
    bool is_esi()      const { return (fd_flags & FD_FLAG_ESI) != 0; }
    uint32_t raw_id()  const { return is_extended() ? (id & ID_EFF_MASK) : (id & ID_SFF_MASK); }
    uint8_t  length()  const { return dlc_to_len(dlc); }
};

} // namespace can
