// mcp2518fd_regs.h - MCP2518FD register addresses and bit definitions
#pragma once

#include <cstdint>

namespace mcp2518fd {

// --- SPI instruction commands (4-bit command in upper nibble of first byte) ---
static constexpr uint8_t SPI_CMD_RESET      = 0x00; // C=0b0000, A=0x000
static constexpr uint8_t SPI_CMD_READ        = 0x03; // C=0b0011
static constexpr uint8_t SPI_CMD_WRITE       = 0x02; // C=0b0010
static constexpr uint8_t SPI_CMD_READ_CRC    = 0x0B; // C=0b1011
static constexpr uint8_t SPI_CMD_WRITE_CRC   = 0x0A; // C=0b1010
static constexpr uint8_t SPI_CMD_WRITE_SAFE  = 0x0C; // C=0b1100

// --- Register addresses ---
// MCP2518FD device SFR (0xE00-0xE17)
static constexpr uint16_t REG_OSC       = 0xE00; // Oscillator Control
static constexpr uint16_t REG_IOCON     = 0xE04; // I/O Control
static constexpr uint16_t REG_CRC       = 0xE08; // CRC Register
static constexpr uint16_t REG_ECCCON    = 0xE0C; // ECC Control
static constexpr uint16_t REG_ECCSTAT   = 0xE10; // ECC Status
static constexpr uint16_t REG_DEVID     = 0xE14; // Device ID

// CAN FD Controller Module SFR (0x000-0x2EF)
static constexpr uint16_t REG_CiCON     = 0x000; // CAN Control
static constexpr uint16_t REG_CiNBTCFG  = 0x004; // Nominal Bit Time Config
static constexpr uint16_t REG_CiDBTCFG  = 0x008; // Data Bit Time Config
static constexpr uint16_t REG_CiTDC     = 0x00C; // Transmitter Delay Compensation
static constexpr uint16_t REG_CiTBC     = 0x010; // Time Base Counter
static constexpr uint16_t REG_CiTSCON   = 0x014; // Time Stamp Control
static constexpr uint16_t REG_CiVEC     = 0x018; // Interrupt Code
static constexpr uint16_t REG_CiINT     = 0x01C; // Interrupt Register
static constexpr uint16_t REG_CiRXIF    = 0x020; // RX Interrupt Flags
static constexpr uint16_t REG_CiRXOVIF  = 0x024; // RX Overflow Interrupt Flags
static constexpr uint16_t REG_CiTXIF    = 0x028; // TX Interrupt Flags
static constexpr uint16_t REG_CiTXATIF  = 0x02C; // TX Attempt Interrupt Flags
static constexpr uint16_t REG_CiTXREQ   = 0x030; // TX Request
static constexpr uint16_t REG_CiTREC    = 0x034; // TX/RX Error Count
static constexpr uint16_t REG_CiBDIAG0  = 0x038; // Bus Diagnostic 0
static constexpr uint16_t REG_CiBDIAG1  = 0x03C; // Bus Diagnostic 1

// TEF (Transmit Event FIFO)
static constexpr uint16_t REG_CiTEFCON  = 0x040; // TEF Control
static constexpr uint16_t REG_CiTEFSTA  = 0x044; // TEF Status
static constexpr uint16_t REG_CiTEFUA   = 0x048; // TEF User Address

// TXQ (Transmit Queue)
static constexpr uint16_t REG_CiTXQCON  = 0x050; // TXQ Control
static constexpr uint16_t REG_CiTXQSTA  = 0x054; // TXQ Status
static constexpr uint16_t REG_CiTXQUA   = 0x058; // TXQ User Address

// FIFOs 1-31 (each FIFO has CON, STA, UA = 3 regs × 4 bytes = 12 bytes stride)
// FIFO m: base = 0x05C + (m-1)*12, m = 1..31
static constexpr uint16_t REG_CiFIFOCON_BASE = 0x05C;
static constexpr uint16_t REG_CiFIFOSTA_BASE = 0x060;
static constexpr uint16_t REG_CiFIFOUA_BASE  = 0x064;
static constexpr uint16_t FIFO_STRIDE         = 12; // bytes between FIFO registers

inline constexpr uint16_t REG_CiFIFOCON(uint8_t m) { return REG_CiFIFOCON_BASE + (m - 1) * FIFO_STRIDE; }
inline constexpr uint16_t REG_CiFIFOSTA(uint8_t m) { return REG_CiFIFOSTA_BASE + (m - 1) * FIFO_STRIDE; }
inline constexpr uint16_t REG_CiFIFOUA(uint8_t m)  { return REG_CiFIFOUA_BASE  + (m - 1) * FIFO_STRIDE; }

// Filter control registers (4 filters per register, 8 registers)
// FLTCONm: m = 0..7 at 0x1D0 + m*4
static constexpr uint16_t REG_CiFLTCON_BASE  = 0x1D0;
inline constexpr uint16_t REG_CiFLTCON(uint8_t m) { return REG_CiFLTCON_BASE + m * 4; }

// Filter object and mask registers (32 filters)
// FLTOBJm: m = 0..31 at 0x1F0 + m*8
// MASKm:   m = 0..31 at 0x1F4 + m*8
static constexpr uint16_t REG_CiFLTOBJ_BASE  = 0x1F0;
static constexpr uint16_t REG_CiMASK_BASE    = 0x1F4;
inline constexpr uint16_t REG_CiFLTOBJ(uint8_t m) { return REG_CiFLTOBJ_BASE + m * 8; }
inline constexpr uint16_t REG_CiMASK(uint8_t m)   { return REG_CiMASK_BASE   + m * 8; }

// RAM start address
static constexpr uint16_t RAM_BASE = 0x400;
static constexpr uint16_t RAM_SIZE = 2048; // 2 KB

// --- OSC register bits ---
static constexpr uint32_t OSC_PLLEN     = (1u << 0);  // PLL Enable
static constexpr uint32_t OSC_OSCDIS    = (1u << 2);  // Oscillator Disable (Sleep)
static constexpr uint32_t OSC_LPMEN     = (1u << 3);  // Low Power Mode Enable
static constexpr uint32_t OSC_SCLKDIV   = (1u << 4);  // System Clock Divisor (/2)
static constexpr uint32_t OSC_CLKODIV_SHIFT = 5;       // CLKODIV[1:0] at bits 6:5
static constexpr uint32_t OSC_CLKODIV_MASK  = (0x3u << OSC_CLKODIV_SHIFT);
static constexpr uint32_t OSC_PLLRDY    = (1u << 8);  // PLL Ready (read-only)
static constexpr uint32_t OSC_OSCRDY    = (1u << 10); // Oscillator Ready (read-only)
static constexpr uint32_t OSC_SCLKRDY   = (1u << 12); // SCLKDIV Ready (read-only)

// --- CiCON register bits ---
// Byte 0 (bits 7:0)
static constexpr uint32_t CiCON_DNCNT_MASK    = 0x1Fu;       // DNCNT[4:0]
static constexpr uint32_t CiCON_ISOCRCEN      = (1u << 5);   // ISO CRC Enable
static constexpr uint32_t CiCON_PXEDIS        = (1u << 6);   // Protocol Exception Disable
// Byte 1 (bits 15:8)
static constexpr uint32_t CiCON_WAKFIL        = (1u << 8);   // Wake-up Filter Enable
static constexpr uint32_t CiCON_WFT_SHIFT     = 9;            // WFT[1:0]
static constexpr uint32_t CiCON_WFT_MASK      = (0x3u << 9);
static constexpr uint32_t CiCON_BUSY          = (1u << 11);  // CAN Module Busy (read-only)
static constexpr uint32_t CiCON_BRSDIS        = (1u << 12);  // Bit Rate Switching Disable
// Byte 2 (bits 23:16)
static constexpr uint32_t CiCON_RTXAT         = (1u << 16);  // Restrict Retransmission Attempts
static constexpr uint32_t CiCON_ESIGM         = (1u << 17);  // Transmit ESI in Gateway Mode
static constexpr uint32_t CiCON_SERR2LOM      = (1u << 18);  // Transition to Listen Only on Error
static constexpr uint32_t CiCON_STEF          = (1u << 19);  // Store in TEF
static constexpr uint32_t CiCON_TXQEN         = (1u << 20);  // Enable TXQ
static constexpr uint32_t CiCON_OPMOD_SHIFT   = 21;           // OPMOD[2:0] (read-only)
static constexpr uint32_t CiCON_OPMOD_MASK    = (0x7u << 21);
// Byte 3 (bits 31:24)
static constexpr uint32_t CiCON_REQOP_SHIFT   = 24;           // REQOP[2:0]
static constexpr uint32_t CiCON_REQOP_MASK    = (0x7u << 24);
static constexpr uint32_t CiCON_ABAT          = (1u << 27);  // Abort All Transmissions
static constexpr uint32_t CiCON_TXBWS_SHIFT   = 28;           // TXBWS[3:0]
static constexpr uint32_t CiCON_TXBWS_MASK    = (0xFu << 28);

// CiCON REQOP / OPMOD values
static constexpr uint8_t CAN_MODE_NORMAL_FD     = 0b000; // Normal CAN FD
static constexpr uint8_t CAN_MODE_SLEEP         = 0b001;
static constexpr uint8_t CAN_MODE_INT_LOOPBACK  = 0b010; // Internal Loopback
static constexpr uint8_t CAN_MODE_LISTEN_ONLY   = 0b011;
static constexpr uint8_t CAN_MODE_CONFIG        = 0b100; // Configuration
static constexpr uint8_t CAN_MODE_EXT_LOOPBACK  = 0b101; // External Loopback
static constexpr uint8_t CAN_MODE_NORMAL_CAN20  = 0b110; // Normal CAN 2.0
static constexpr uint8_t CAN_MODE_RESTRICTED    = 0b111; // Restricted Operation

// --- CiNBTCFG - Nominal Bit Time Configuration ---
// SJW[6:0] bits 6:0, TSEG2[6:0] bits 14:8, TSEG1[7:0] bits 23:16, BRP[7:0] bits 31:24
static constexpr uint32_t NBTCFG_SJW_SHIFT    = 0;
static constexpr uint32_t NBTCFG_SJW_MASK     = (0x7Fu << 0);
static constexpr uint32_t NBTCFG_TSEG2_SHIFT  = 8;
static constexpr uint32_t NBTCFG_TSEG2_MASK   = (0x7Fu << 8);
static constexpr uint32_t NBTCFG_TSEG1_SHIFT  = 16;
static constexpr uint32_t NBTCFG_TSEG1_MASK   = (0xFFu << 16);
static constexpr uint32_t NBTCFG_BRP_SHIFT    = 24;
static constexpr uint32_t NBTCFG_BRP_MASK     = (0xFFu << 24);

// --- CiDBTCFG - Data Bit Time Configuration ---
// SJW[3:0] bits 3:0, TSEG2[3:0] bits 11:8, TSEG1[4:0] bits 20:16, BRP[7:0] bits 31:24
static constexpr uint32_t DBTCFG_SJW_SHIFT    = 0;
static constexpr uint32_t DBTCFG_SJW_MASK     = (0x0Fu << 0);
static constexpr uint32_t DBTCFG_TSEG2_SHIFT  = 8;
static constexpr uint32_t DBTCFG_TSEG2_MASK   = (0x0Fu << 8);
static constexpr uint32_t DBTCFG_TSEG1_SHIFT  = 16;
static constexpr uint32_t DBTCFG_TSEG1_MASK   = (0x1Fu << 16);
static constexpr uint32_t DBTCFG_BRP_SHIFT    = 24;
static constexpr uint32_t DBTCFG_BRP_MASK     = (0xFFu << 24);

// --- CiTDC - Transmitter Delay Compensation ---
static constexpr uint32_t TDC_TDCV_SHIFT      = 0;   // TDCV[5:0] bits 5:0 (read-only measured)
static constexpr uint32_t TDC_TDCV_MASK       = (0x3Fu << 0);
static constexpr uint32_t TDC_TDCO_SHIFT      = 8;   // TDCO[6:0] bits 14:8
static constexpr uint32_t TDC_TDCO_MASK       = (0x7Fu << 8);
static constexpr uint32_t TDC_TDCMOD_SHIFT    = 16;  // TDCMOD[1:0] bits 17:16
static constexpr uint32_t TDC_TDCMOD_MASK     = (0x3u << 16);
static constexpr uint32_t TDC_SID11EN         = (1u << 24); // 12-bit SID in CAN FD
static constexpr uint32_t TDC_EDGFLTEN        = (1u << 25); // Edge Filter Enable

// --- CiTSCON - Time Stamp Control ---
static constexpr uint32_t TSCON_TBCPRE_SHIFT  = 0;   // TBCPRE[9:0] bits 9:0
static constexpr uint32_t TSCON_TBCPRE_MASK   = (0x3FFu << 0);
static constexpr uint32_t TSCON_TBCEN         = (1u << 16); // Time Base Counter Enable
static constexpr uint32_t TSCON_TSEOF         = (1u << 17); // Timestamp at EOF
static constexpr uint32_t TSCON_TSRES         = (1u << 18); // Timestamp Prescaler Reset

// --- CiFIFOCONm register bits (applies to FIFO 1-31, also TXQ with some differences) ---
// Byte 0 (bits 7:0)
static constexpr uint32_t FIFOCON_TFNRFNIE    = (1u << 0); // TX FIFO Not Full / RX FIFO Not Empty IE
static constexpr uint32_t FIFOCON_TFHRFHIE    = (1u << 1); // TX FIFO Half Empty / RX FIFO Half Full IE
static constexpr uint32_t FIFOCON_TFERFFIE    = (1u << 2); // TX FIFO Empty / RX FIFO Full IE
static constexpr uint32_t FIFOCON_RXOVIE      = (1u << 3); // RX FIFO Overflow IE
static constexpr uint32_t FIFOCON_TXATIE      = (1u << 4); // TX Attempt IE
static constexpr uint32_t FIFOCON_RXTSEN      = (1u << 5); // RX FIFO Timestamp Enable
static constexpr uint32_t FIFOCON_RTREN       = (1u << 6); // Auto RTR Enable
static constexpr uint32_t FIFOCON_TXEN        = (1u << 7); // TX FIFO Enable (1=TX, 0=RX)
// Byte 1 (bits 15:8)
static constexpr uint32_t FIFOCON_UINC        = (1u << 8);  // Increment Head/Tail
static constexpr uint32_t FIFOCON_TXREQ       = (1u << 9);  // TX Request (set to transmit)
static constexpr uint32_t FIFOCON_FRESET      = (1u << 10); // FIFO Reset
// Byte 2 (bits 23:16)
static constexpr uint32_t FIFOCON_TXPRI_SHIFT = 16;          // TXPRI[4:0] bits 20:16
static constexpr uint32_t FIFOCON_TXPRI_MASK  = (0x1Fu << 16);
static constexpr uint32_t FIFOCON_TXAT_SHIFT  = 21;          // TXAT[1:0] bits 22:21
static constexpr uint32_t FIFOCON_TXAT_MASK   = (0x3u << 21);
// Byte 3 (bits 31:24)
static constexpr uint32_t FIFOCON_FSIZE_SHIFT = 24;          // FSIZE[4:0] bits 28:24 (FIFO size - 1)
static constexpr uint32_t FIFOCON_FSIZE_MASK  = (0x1Fu << 24);
static constexpr uint32_t FIFOCON_PLSIZE_SHIFT = 29;         // PLSIZE[2:0] bits 31:29
static constexpr uint32_t FIFOCON_PLSIZE_MASK = (0x7u << 29);

// PLSIZE values (payload size)
static constexpr uint8_t PLSIZE_8   = 0; // 8 data bytes
static constexpr uint8_t PLSIZE_12  = 1; // 12 data bytes
static constexpr uint8_t PLSIZE_16  = 2; // 16 data bytes
static constexpr uint8_t PLSIZE_20  = 3; // 20 data bytes
static constexpr uint8_t PLSIZE_24  = 4; // 24 data bytes
static constexpr uint8_t PLSIZE_32  = 5; // 32 data bytes
static constexpr uint8_t PLSIZE_48  = 6; // 48 data bytes
static constexpr uint8_t PLSIZE_64  = 7; // 64 data bytes

// --- CiFIFOSTAm register bits ---
static constexpr uint32_t FIFOSTA_TFNRFNIF    = (1u << 0); // TX Not Full / RX Not Empty IF
static constexpr uint32_t FIFOSTA_TFHRFHIF    = (1u << 1); // TX Half Empty / RX Half Full IF
static constexpr uint32_t FIFOSTA_TFERFFIF    = (1u << 2); // TX Empty / RX Full IF
static constexpr uint32_t FIFOSTA_RXOVIF      = (1u << 3); // RX Overflow IF
static constexpr uint32_t FIFOSTA_TXATIF      = (1u << 4); // TX Attempt IF
static constexpr uint32_t FIFOSTA_TXERR       = (1u << 5); // TX Error
static constexpr uint32_t FIFOSTA_TXLARB      = (1u << 6); // TX Lost Arbitration
static constexpr uint32_t FIFOSTA_TXABT       = (1u << 7); // TX Aborted
static constexpr uint32_t FIFOSTA_FIFOCI_SHIFT = 8;          // FIFOCI[4:0] bits 12:8 (FIFO CI)
static constexpr uint32_t FIFOSTA_FIFOCI_MASK = (0x1Fu << 8);

// --- CiINT register bits ---
// Interrupt flags (lower 16 bits, byte 0-1)
static constexpr uint32_t CiINT_TXIF       = (1u << 0);  // TX Interrupt Flag
static constexpr uint32_t CiINT_RXIF       = (1u << 1);  // RX Interrupt Flag
static constexpr uint32_t CiINT_TBCIF      = (1u << 2);  // Time Base Counter IF
static constexpr uint32_t CiINT_MODIF      = (1u << 3);  // Mode Change IF
static constexpr uint32_t CiINT_TEFIF      = (1u << 4);  // TEF IF
static constexpr uint32_t CiINT_ECCIF      = (1u << 8);  // ECC Error IF
static constexpr uint32_t CiINT_SPICRCIF   = (1u << 9);  // SPI CRC Error IF
static constexpr uint32_t CiINT_TXATIF     = (1u << 10); // TX Attempt IF
static constexpr uint32_t CiINT_RXOVIF     = (1u << 11); // RX Overflow IF
static constexpr uint32_t CiINT_SERRIF     = (1u << 12); // System Error IF
static constexpr uint32_t CiINT_CERRIF     = (1u << 13); // CAN Bus Error IF
static constexpr uint32_t CiINT_WAKIF      = (1u << 14); // Wake-up IF
static constexpr uint32_t CiINT_IVMIF      = (1u << 15); // Invalid Message IF

// Interrupt enables (upper 16 bits, byte 2-3)
static constexpr uint32_t CiINT_TXIE       = (1u << 16); // TX Interrupt Enable
static constexpr uint32_t CiINT_RXIE       = (1u << 17); // RX Interrupt Enable
static constexpr uint32_t CiINT_TBCIE      = (1u << 18); // Time Base Counter IE
static constexpr uint32_t CiINT_MODIE      = (1u << 19); // Mode Change IE
static constexpr uint32_t CiINT_TEFIE      = (1u << 20); // TEF IE
static constexpr uint32_t CiINT_ECCIE      = (1u << 24); // ECC Error IE
static constexpr uint32_t CiINT_SPICRCIE   = (1u << 25); // SPI CRC Error IE
static constexpr uint32_t CiINT_TXATIE     = (1u << 26); // TX Attempt IE
static constexpr uint32_t CiINT_RXOVIE     = (1u << 27); // RX Overflow IE
static constexpr uint32_t CiINT_SERRIE     = (1u << 28); // System Error IE
static constexpr uint32_t CiINT_CERRIE     = (1u << 29); // CAN Bus Error IE
static constexpr uint32_t CiINT_WAKIE      = (1u << 30); // Wake-up IE
static constexpr uint32_t CiINT_IVMIE      = (1u << 31); // Invalid Message IE

// --- CiTREC register bits ---
static constexpr uint32_t TREC_REC_SHIFT   = 0;           // REC[7:0]
static constexpr uint32_t TREC_REC_MASK    = (0xFFu << 0);
static constexpr uint32_t TREC_TEC_SHIFT   = 8;           // TEC[7:0]
static constexpr uint32_t TREC_TEC_MASK    = (0xFFu << 8);
static constexpr uint32_t TREC_EWARN       = (1u << 16);  // Error Warning
static constexpr uint32_t TREC_RXWARN      = (1u << 17);  // RX Warning
static constexpr uint32_t TREC_TXWARN      = (1u << 18);  // TX Warning
static constexpr uint32_t TREC_RXBP        = (1u << 19);  // RX Bus Passive
static constexpr uint32_t TREC_TXBP        = (1u << 20);  // TX Bus Passive
static constexpr uint32_t TREC_TXBO        = (1u << 21);  // TX Bus Off

// --- TX/RX message object layout ---
// Word T0/R0: ID fields
static constexpr uint32_t TXOBJ_SID_SHIFT     = 0;           // SID[10:0] bits 10:0
static constexpr uint32_t TXOBJ_SID_MASK      = (0x7FFu << 0);
static constexpr uint32_t TXOBJ_EID_SHIFT     = 11;          // EID[17:0] bits 28:11
static constexpr uint32_t TXOBJ_EID_MASK      = (0x3FFFFu << 11);
static constexpr uint32_t TXOBJ_SID11         = (1u << 29);  // Extended SID bit 11

// Word T1/R1: Control fields
static constexpr uint32_t TXOBJ_DLC_SHIFT     = 0;           // DLC[3:0] bits 3:0
static constexpr uint32_t TXOBJ_DLC_MASK      = (0xFu << 0);
static constexpr uint32_t TXOBJ_IDE           = (1u << 4);   // Extended Identifier
static constexpr uint32_t TXOBJ_RTR           = (1u << 5);   // Remote Transmission Request
static constexpr uint32_t TXOBJ_BRS           = (1u << 6);   // Bit Rate Switch
static constexpr uint32_t TXOBJ_FDF           = (1u << 7);   // FD Frame
static constexpr uint32_t TXOBJ_ESI           = (1u << 8);   // Error Status Indicator
static constexpr uint32_t TXOBJ_SEQ_SHIFT     = 9;           // SEQ[22:0] bits 31:9
static constexpr uint32_t TXOBJ_SEQ_MASK      = (0x7FFFFFu << 9);

// RX message R1 extras
static constexpr uint32_t RXOBJ_FILHIT_SHIFT  = 11;          // FILHIT[4:0] in R1 bits 15:11
static constexpr uint32_t RXOBJ_FILHIT_MASK   = (0x1Fu << 11);

// Word R2: Receive Timestamp (32-bit)
// R2 = RXMSGTS[31:0]

// Data starts at word T2 (TX) or R3/R4 (RX with/without timestamp)
static constexpr uint8_t TX_DATA_OFFSET       = 8;  // bytes from start of TX object to data
static constexpr uint8_t RX_DATA_OFFSET_NO_TS = 8;  // without timestamp
static constexpr uint8_t RX_DATA_OFFSET_TS    = 12; // with timestamp (R2 = timestamp)

// --- CiFLTCONm bits (4 filters per register) ---
// Each filter n in register m: byte position = (n % 4)
// Bits: FLTEN (bit 7), FnBP[4:0] (bits 4:0) = FIFO pointer
static constexpr uint8_t FLTCON_FLTEN         = (1u << 7);  // Filter Enable
static constexpr uint8_t FLTCON_FBP_MASK      = 0x1Fu;       // FIFO pointer (0-31)

// --- CiFLTOBJm bits ---
static constexpr uint32_t FLTOBJ_SID_SHIFT    = 0;
static constexpr uint32_t FLTOBJ_SID_MASK     = (0x7FFu << 0);
static constexpr uint32_t FLTOBJ_EID_SHIFT    = 11;
static constexpr uint32_t FLTOBJ_EID_MASK     = (0x3FFFFu << 11);
static constexpr uint32_t FLTOBJ_SID11        = (1u << 29);
static constexpr uint32_t FLTOBJ_EXIDE        = (1u << 30);  // Extended ID Filter Enable

} // namespace mcp2518fd
