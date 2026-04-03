// mcp2518fd.h - MCP2518FD CAN-FD controller SPI driver interface
#pragma once

#include <cstdint>
#include <cstddef>
#include "can_frame.h"
#include "mcp2518fd_regs.h"

namespace mcp2518fd {

// --- Pin configuration ---
struct PinConfig {
    uint8_t spi_inst;   // SPI instance (0 or 1)
    uint8_t pin_miso;
    uint8_t pin_mosi;
    uint8_t pin_sck;
    uint8_t pin_cs;
    uint8_t pin_int;    // interrupt pin (active low)
};

// --- Bit timing configuration ---
// Maps directly to CiNBTCFG / CiDBTCFG register fields.
// Values are register values (0-based), NOT the actual segment lengths.
// Actual length = register value + 1.
struct BitTimingConfig {
    uint8_t brp;        // Baud Rate Prescaler (0-255): TQ = (brp+1)/Fsys
    uint8_t tseg1;      // Time Segment 1 = Prop + Phase1 (0-255 nominal, 0-31 data)
    uint8_t tseg2;      // Time Segment 2 = Phase2 (0-127 nominal, 0-15 data)
    uint8_t sjw;        // Synchronization Jump Width (0-127 nominal, 0-15 data)
};

// --- FIFO assignment ---
static constexpr uint8_t TX_FIFO  = 1;  // FIFO 1 used for TX
static constexpr uint8_t RX_FIFO  = 2;  // FIFO 2 used for RX

// --- Driver class ---
class Driver {
public:
    explicit Driver(const PinConfig& pins);

    // Initialize SPI peripheral and MCP2518FD hardware.
    // Configures oscillator, bit timing, FIFOs, filters, and enters config mode.
    bool init();

    // Configure nominal (arbitration) bit timing
    void set_nominal_bittiming(const BitTimingConfig& bt);

    // Configure data phase bit timing (CAN-FD)
    void set_data_bittiming(const BitTimingConfig& bt);

    // Request operation mode change and wait for it to take effect
    // mode: one of CAN_MODE_* constants from mcp2518fd_regs.h
    bool set_mode(uint8_t mode);

    // Get current operation mode (OPMOD field from CiCON)
    uint8_t get_mode();

    // Transmit a CAN-FD frame. Returns true if written to TX FIFO successfully.
    bool transmit(const can::FdFrame& frame);

    // Check if a received frame is available in the RX FIFO
    bool rx_available();

    // Receive a CAN-FD frame from RX FIFO. Returns true if a frame was read.
    bool receive(can::FdFrame& frame);

    // Read error counters (TEC, REC) from CiTREC
    void read_error_counters(uint8_t& tec, uint8_t& rec);

    // Read the interrupt flags register (CiINT)
    uint32_t read_interrupts();

    // Read device ID register
    uint8_t read_device_id();

private:
    PinConfig pins_;

    // Get the RP2350 SPI hardware instance pointer from spi_inst number
    void* spi_hw() const;

    // --- Low-level SPI operations ---
    void cs_select();
    void cs_deselect();
    void spi_reset();
    uint32_t read_register(uint16_t addr);
    void write_register(uint16_t addr, uint32_t value);
    void read_bytes(uint16_t addr, uint8_t* buf, size_t len);
    void write_bytes(uint16_t addr, const uint8_t* buf, size_t len);

    // --- FIFO helpers ---
    void configure_tx_fifo();
    void configure_rx_fifo();
    void configure_filter_accept_all();
};

} // namespace mcp2518fd
