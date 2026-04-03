// mcp2518fd.cpp - MCP2518FD CAN-FD controller SPI driver implementation

#include "mcp2518fd.h"
#include "mcp2518fd_regs.h"
#include "hardware/spi.h"
#include "hardware/gpio.h"
#include "pico/stdlib.h"
#include <cstring>

// SPI clock frequency for MCP2518FD (max 20 MHz, use 10 MHz for safety)
static constexpr uint32_t SPI_FREQ_HZ = 10'000'000;

namespace mcp2518fd {

Driver::Driver(const PinConfig& pins) : pins_(pins) {}

// Get the RP2350 SPI hardware instance from config
void* Driver::spi_hw() const {
    return (pins_.spi_inst == 0) ? static_cast<void*>(spi0) : static_cast<void*>(spi1);
}

static inline spi_inst_t* to_spi(void* hw) {
    return static_cast<spi_inst_t*>(hw);
}

void Driver::cs_select() {
    gpio_put(pins_.pin_cs, false);
}

void Driver::cs_deselect() {
    gpio_put(pins_.pin_cs, true);
}

bool Driver::init() {
    // --- Initialize SPI peripheral ---
    spi_init(to_spi(spi_hw()), SPI_FREQ_HZ);
    spi_set_format(to_spi(spi_hw()), 8, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);

    // Configure GPIO pins for SPI
    gpio_set_function(pins_.pin_miso, GPIO_FUNC_SPI);
    gpio_set_function(pins_.pin_mosi, GPIO_FUNC_SPI);
    gpio_set_function(pins_.pin_sck,  GPIO_FUNC_SPI);

    // CS pin: manual control (active low)
    gpio_init(pins_.pin_cs);
    gpio_set_dir(pins_.pin_cs, GPIO_OUT);
    gpio_put(pins_.pin_cs, true); // deasserted

    // INT pin: input (active low)
    gpio_init(pins_.pin_int);
    gpio_set_dir(pins_.pin_int, GPIO_IN);
    gpio_pull_up(pins_.pin_int);

    // --- Reset MCP2518FD ---
    spi_reset();
    sleep_ms(3);

    // --- Verify we can read the device: check OSC register ---
    // After reset, OSCRDY should go high. Wait for it.
    for (int i = 0; i < 100; i++) {
        uint32_t osc = read_register(REG_OSC);
        if (osc & OSC_OSCRDY) {
            break;
        }
        sleep_ms(1);
    }

    // Verify we are in Configuration mode (OPMOD = 100 = 4)
    uint32_t con = read_register(REG_CiCON);
    uint8_t opmod = (con & CiCON_OPMOD_MASK) >> CiCON_OPMOD_SHIFT;
    if (opmod != CAN_MODE_CONFIG) {
        return false; // Not in config mode after reset — hardware error
    }

    // --- Configure CiCON ---
    // Enable: ISO CRC, protocol exception disable, store in TEF off, TXQ off
    con = CiCON_ISOCRCEN | CiCON_PXEDIS
        | (static_cast<uint32_t>(CAN_MODE_CONFIG) << CiCON_REQOP_SHIFT);
    write_register(REG_CiCON, con);

    // --- Configure FIFOs ---
    configure_tx_fifo();
    configure_rx_fifo();

    // --- Configure filter to accept all messages into RX FIFO ---
    configure_filter_accept_all();

    // --- Enable interrupts: RX and TX ---
    uint32_t int_reg = CiINT_RXIE | CiINT_TXIE;
    write_register(REG_CiINT, int_reg);

    return true;
}

void Driver::set_nominal_bittiming(const BitTimingConfig& bt) {
    uint32_t val = (static_cast<uint32_t>(bt.brp)   << NBTCFG_BRP_SHIFT)
                 | (static_cast<uint32_t>(bt.tseg1)  << NBTCFG_TSEG1_SHIFT)
                 | (static_cast<uint32_t>(bt.tseg2)  << NBTCFG_TSEG2_SHIFT)
                 | (static_cast<uint32_t>(bt.sjw)    << NBTCFG_SJW_SHIFT);
    write_register(REG_CiNBTCFG, val);
}

void Driver::set_data_bittiming(const BitTimingConfig& bt) {
    uint32_t val = (static_cast<uint32_t>(bt.brp)   << DBTCFG_BRP_SHIFT)
                 | (static_cast<uint32_t>(bt.tseg1)  << DBTCFG_TSEG1_SHIFT)
                 | (static_cast<uint32_t>(bt.tseg2)  << DBTCFG_TSEG2_SHIFT)
                 | (static_cast<uint32_t>(bt.sjw)    << DBTCFG_SJW_SHIFT);
    write_register(REG_CiDBTCFG, val);

    // Enable auto TDC
    uint32_t tdc = (0x2u << TDC_TDCMOD_SHIFT); // Auto mode
    write_register(REG_CiTDC, tdc);
}

bool Driver::set_mode(uint8_t mode) {
    // Read current CiCON, modify REQOP, write back
    uint32_t con = read_register(REG_CiCON);
    con &= ~CiCON_REQOP_MASK;
    con |= (static_cast<uint32_t>(mode) << CiCON_REQOP_SHIFT);
    write_register(REG_CiCON, con);

    // Wait for OPMOD to match requested mode (timeout ~100ms)
    for (int i = 0; i < 100; i++) {
        con = read_register(REG_CiCON);
        uint8_t opmod = (con & CiCON_OPMOD_MASK) >> CiCON_OPMOD_SHIFT;
        if (opmod == mode) {
            return true;
        }
        sleep_ms(1);
    }
    return false; // Timeout
}

uint8_t Driver::get_mode() {
    uint32_t con = read_register(REG_CiCON);
    return static_cast<uint8_t>((con & CiCON_OPMOD_MASK) >> CiCON_OPMOD_SHIFT);
}

bool Driver::transmit(const can::FdFrame& frame) {
    // Check TX FIFO has space (TFNRFNIF = TX Not Full)
    uint32_t sta = read_register(REG_CiFIFOSTA(TX_FIFO));
    if (!(sta & FIFOSTA_TFNRFNIF)) {
        return false; // TX FIFO full
    }

    // Get user address (points to next free TX slot in RAM)
    uint32_t ua = read_register(REG_CiFIFOUA(TX_FIFO));
    uint16_t ram_addr = static_cast<uint16_t>(RAM_BASE + ua);

    // Build TX message object
    // Word T0: SID, EID
    uint32_t t0 = 0;
    if (frame.is_extended()) {
        uint32_t raw = frame.raw_id();
        t0 = (raw & 0x7FFu)                       // SID[10:0]
           | ((raw >> 11) & 0x3FFFFu) << 11;       // EID[17:0]
    } else {
        t0 = frame.raw_id() & 0x7FFu;              // SID[10:0]
    }

    // Word T1: DLC, IDE, RTR, BRS, FDF, ESI
    uint32_t t1 = (static_cast<uint32_t>(frame.dlc) & 0xFu);
    if (frame.is_extended()) t1 |= TXOBJ_IDE;
    if (frame.is_rtr())      t1 |= TXOBJ_RTR;
    if (frame.is_brs())      t1 |= TXOBJ_BRS;
    // Always set FDF for CAN-FD frames (when data > 8 or BRS set)
    if (frame.dlc > 8 || frame.is_brs()) {
        t1 |= TXOBJ_FDF;
    }

    // Write T0 and T1 to RAM
    uint8_t header[8];
    header[0] = static_cast<uint8_t>(t0);
    header[1] = static_cast<uint8_t>(t0 >> 8);
    header[2] = static_cast<uint8_t>(t0 >> 16);
    header[3] = static_cast<uint8_t>(t0 >> 24);
    header[4] = static_cast<uint8_t>(t1);
    header[5] = static_cast<uint8_t>(t1 >> 8);
    header[6] = static_cast<uint8_t>(t1 >> 16);
    header[7] = static_cast<uint8_t>(t1 >> 24);
    write_bytes(ram_addr, header, 8);

    // Write data bytes
    uint8_t data_len = frame.length();
    if (data_len > 0) {
        write_bytes(ram_addr + TX_DATA_OFFSET, frame.data.data(), data_len);
    }

    // Set UINC (increment FIFO head) and TXREQ (request transmission)
    uint32_t fifocon = read_register(REG_CiFIFOCON(TX_FIFO));
    fifocon |= FIFOCON_UINC | FIFOCON_TXREQ;
    write_register(REG_CiFIFOCON(TX_FIFO), fifocon);

    return true;
}

bool Driver::rx_available() {
    // Check RX FIFO Not Empty flag (TFNRFNIF for RX = Not Empty)
    uint32_t sta = read_register(REG_CiFIFOSTA(RX_FIFO));
    return (sta & FIFOSTA_TFNRFNIF) != 0;
}

bool Driver::receive(can::FdFrame& frame) {
    // Check if data available
    uint32_t sta = read_register(REG_CiFIFOSTA(RX_FIFO));
    if (!(sta & FIFOSTA_TFNRFNIF)) {
        return false;
    }

    // Get user address (points to next message to read in RAM)
    uint32_t ua = read_register(REG_CiFIFOUA(RX_FIFO));
    uint16_t ram_addr = static_cast<uint16_t>(RAM_BASE + ua);

    // Read R0, R1 (8 bytes header)
    uint8_t header[8] = {};
    read_bytes(ram_addr, header, 8);

    uint32_t r0 = static_cast<uint32_t>(header[0])
                | (static_cast<uint32_t>(header[1]) << 8)
                | (static_cast<uint32_t>(header[2]) << 16)
                | (static_cast<uint32_t>(header[3]) << 24);

    uint32_t r1 = static_cast<uint32_t>(header[4])
                | (static_cast<uint32_t>(header[5]) << 8)
                | (static_cast<uint32_t>(header[6]) << 16)
                | (static_cast<uint32_t>(header[7]) << 24);

    // Parse ID
    bool ide = (r1 & TXOBJ_IDE) != 0;
    if (ide) {
        uint32_t sid = r0 & TXOBJ_SID_MASK;
        uint32_t eid = (r0 & TXOBJ_EID_MASK) >> TXOBJ_EID_SHIFT;
        frame.id = (sid | (eid << 11)) | can::ID_EFF_FLAG;
    } else {
        frame.id = r0 & TXOBJ_SID_MASK;
    }

    if (r1 & TXOBJ_RTR) frame.id |= can::ID_RTR_FLAG;

    // Parse DLC and flags
    frame.dlc = static_cast<uint8_t>(r1 & TXOBJ_DLC_MASK);
    frame.fd_flags = 0;
    if (r1 & TXOBJ_BRS) frame.fd_flags |= can::FD_FLAG_BRS;
    if (r1 & TXOBJ_ESI) frame.fd_flags |= can::FD_FLAG_ESI;

    // Determine data offset: if timestamp enabled, R2 is timestamp (4 bytes)
    // We configured RX FIFO with timestamp, so data starts at offset 12
    uint8_t data_offset = RX_DATA_OFFSET_TS;

    // Read data bytes
    uint8_t data_len = frame.length();
    frame.data.fill(0);
    if (data_len > 0) {
        read_bytes(ram_addr + data_offset, frame.data.data(), data_len);
    }

    // Set UINC to release this message slot
    uint32_t fifocon = read_register(REG_CiFIFOCON(RX_FIFO));
    fifocon |= FIFOCON_UINC;
    write_register(REG_CiFIFOCON(RX_FIFO), fifocon);

    return true;
}

void Driver::read_error_counters(uint8_t& tec, uint8_t& rec) {
    uint32_t trec = read_register(REG_CiTREC);
    rec = static_cast<uint8_t>((trec & TREC_REC_MASK) >> TREC_REC_SHIFT);
    tec = static_cast<uint8_t>((trec & TREC_TEC_MASK) >> TREC_TEC_SHIFT);
}

uint32_t Driver::read_interrupts() {
    return read_register(REG_CiINT);
}

uint8_t Driver::read_device_id() {
    uint32_t devid = read_register(REG_DEVID);
    return static_cast<uint8_t>((devid >> 4) & 0x0F);
}

// --- Low-level SPI ---

void Driver::spi_reset() {
    // SPI RESET instruction: CMD=0x00, ADDR=0x000
    uint8_t buf[2] = {
        static_cast<uint8_t>((SPI_CMD_RESET << 4) | 0x00), // cmd=0000, addr[11:8]=0
        0x00  // addr[7:0]=0
    };
    cs_select();
    spi_write_blocking(to_spi(spi_hw()), buf, 2);
    cs_deselect();
    sleep_ms(5); // Wait for reset to complete
}

uint32_t Driver::read_register(uint16_t addr) {
    uint8_t cmd[2] = {
        static_cast<uint8_t>((SPI_CMD_READ << 4) | ((addr >> 8) & 0x0F)),
        static_cast<uint8_t>(addr & 0xFF)
    };
    uint8_t data[4] = {};

    cs_select();
    spi_write_blocking(to_spi(spi_hw()), cmd, 2);
    spi_read_blocking(to_spi(spi_hw()), 0x00, data, 4);
    cs_deselect();

    // MCP2518FD is little-endian: LSB at lower address
    return static_cast<uint32_t>(data[0])
         | (static_cast<uint32_t>(data[1]) << 8)
         | (static_cast<uint32_t>(data[2]) << 16)
         | (static_cast<uint32_t>(data[3]) << 24);
}

void Driver::write_register(uint16_t addr, uint32_t value) {
    uint8_t buf[6] = {
        static_cast<uint8_t>((SPI_CMD_WRITE << 4) | ((addr >> 8) & 0x0F)),
        static_cast<uint8_t>(addr & 0xFF),
        static_cast<uint8_t>(value & 0xFF),
        static_cast<uint8_t>((value >> 8) & 0xFF),
        static_cast<uint8_t>((value >> 16) & 0xFF),
        static_cast<uint8_t>((value >> 24) & 0xFF),
    };

    cs_select();
    spi_write_blocking(to_spi(spi_hw()), buf, 6);
    cs_deselect();
}

void Driver::read_bytes(uint16_t addr, uint8_t* buf, size_t len) {
    uint8_t cmd[2] = {
        static_cast<uint8_t>((SPI_CMD_READ << 4) | ((addr >> 8) & 0x0F)),
        static_cast<uint8_t>(addr & 0xFF)
    };

    cs_select();
    spi_write_blocking(to_spi(spi_hw()), cmd, 2);
    spi_read_blocking(to_spi(spi_hw()), 0x00, buf, len);
    cs_deselect();
}

void Driver::write_bytes(uint16_t addr, const uint8_t* buf, size_t len) {
    uint8_t cmd[2] = {
        static_cast<uint8_t>((SPI_CMD_WRITE << 4) | ((addr >> 8) & 0x0F)),
        static_cast<uint8_t>(addr & 0xFF)
    };

    cs_select();
    spi_write_blocking(to_spi(spi_hw()), cmd, 2);
    spi_write_blocking(to_spi(spi_hw()), buf, len);
    cs_deselect();
}

// --- FIFO configuration helpers ---

void Driver::configure_tx_fifo() {
    // FIFO 1 as TX: 8 messages deep, 64 byte payload (CAN-FD max)
    uint32_t fifocon = FIFOCON_TXEN                          // TX FIFO
                     | (static_cast<uint32_t>(PLSIZE_64) << FIFOCON_PLSIZE_SHIFT)  // 64-byte payload
                     | (7u << FIFOCON_FSIZE_SHIFT)           // 8 messages (FSIZE = 7 means 8)
                     | (0u << FIFOCON_TXPRI_SHIFT);          // Priority 0
    write_register(REG_CiFIFOCON(TX_FIFO), fifocon);
}

void Driver::configure_rx_fifo() {
    // FIFO 2 as RX: 16 messages deep, 64 byte payload, timestamp enabled
    uint32_t fifocon = 0                                     // TXEN=0 => RX FIFO
                     | FIFOCON_RXTSEN                        // Enable RX timestamps
                     | FIFOCON_TFNRFNIE                      // Interrupt on not empty
                     | (static_cast<uint32_t>(PLSIZE_64) << FIFOCON_PLSIZE_SHIFT)  // 64-byte payload
                     | (15u << FIFOCON_FSIZE_SHIFT);         // 16 messages (FSIZE = 15 means 16)
    write_register(REG_CiFIFOCON(RX_FIFO), fifocon);
}

void Driver::configure_filter_accept_all() {
    // Filter 0: accept all messages, point to RX_FIFO
    // FLTOBJm = 0 (match anything when mask = 0)
    write_register(REG_CiFLTOBJ(0), 0x00000000);
    // MASKm = 0 (don't care about any bits)
    write_register(REG_CiMASK(0), 0x00000000);

    // FLTCONm: enable filter 0, point to RX_FIFO
    // Filter 0 is in byte 0 of FLTCON0
    uint32_t fltcon = read_register(REG_CiFLTCON(0));
    fltcon &= ~0xFFu; // Clear byte 0
    fltcon |= FLTCON_FLTEN | (RX_FIFO & FLTCON_FBP_MASK);
    write_register(REG_CiFLTCON(0), fltcon);
}

} // namespace mcp2518fd
