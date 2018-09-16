#ifndef MCP2515_DEF_H_
#define MCP2515_DEF_H_

// spi instruction set
#define MCP_SPI_RESET 0xc0
#define MCP_SPI_READ 0x03
#define MCP_SPI_READ_RX0 0x90
#define MCP_SPI_READ_RX1 0x94
#define MCP_SPI_WRITE 0x02
#define MCP_SPI_LOAD_TX 0x40
#define MCP_SPI_LOAD_TX0 0x41
#define MCP_SPI_LOAD_TX1 0x42
#define MCP_SPI_LOAD_TX2 0x44
#define MCP_SPI_RTS 0x80
#define MCP_SPI_RTS_TX0 0x81
#define MCP_SPI_RTS_TX1 0x82
#define MCP_SPI_RTS_TX2 0x84
#define MCP_SPI_READ_STATUS 0xa0
#define MCP_SPI_RX_STATUS 0xb0
#define MCP_SPI_BIT_MOD 0x05

// spi instruction set bit values and masks

// MCP_SPI_READ_STATUS
#define MCP_CANINTF_RX0IF 0
#define MCP_CANINTF_RX1IF 1
#define MCP_TXB0CNTRL_TXREQ 2
#define MCP_CANINTF_TX0IF 3
#define MCP_TXB1CNTRL_TXREQ 4
#define MCP_CANINTF_TX1IF 5
#define MCP_TXB2CNTRL_TXREQ 6
#define MCP_CANINTF_TX2IF 7

// MCP_SPI_RX_STATUS
#define MCP_MSG_RECV_MASK 0xc0
#define MCP_REMOTE_FRAME 3
#define MCP_EXT_DATA_FRAME 4
#define MCP_MSG_MASK 0xc
#define MCP_MSG_RXB0 6
#define MCP_MSG_RXB1 7
#define MCP_FILTER_MATCH_MASK 0x7

// registers
#define MCP_BFPCTRL 0xc
#define MCP_TXRTSCTRL 0xd
#define MCP_CANSTAT 0xe
#define MCP_CANCTRL 0xf
#define MCP_TEC 0x1c
#define MCP_REC 0x1d
#define MCP_CNF3 0x28
#define MCP_CNF2 0x29
#define MCP_CNF1 0x2a
#define MCP_CANINTE 0x2b
#define MCP_CANINTF 0x2c
#define MCP_EFLG 0x2d
#define MCP_TXB0CTRL 0x30
#define MCP_TXB1CTRL 0x40
#define MCP_TXB2CTRL 0x50
#define MCP_RXB0CTRL 0x60
#define MCP_RXB1CTRL 0x70

// data registers
#define MCP_RXF0SIDH 0x00
#define MCP_RXF0SIDL 0x01
#define MCP_RXF0EID8 0x02
#define MCP_RXF0EID0 0x03

#define MCP_RXF1SIDH 0x04
#define MCP_RXF1SIDL 0x05
#define MCP_RXF1EID8 0x06
#define MCP_RXF1EID0 0x07

#define MCP_RXF2SIDH 0x08
#define MCP_RXF2SIDL 0x09
#define MCP_RXF2EID8 0x0a
#define MCP_RXF2EID0 0x0b

#define MCP_RXF3SIDH 0x10
#define MCP_RXF3SIDL 0x11
#define MCP_RXF3EID8 0x12
#define MCP_RXF3EID0 0x13

#define MCP_RXF4SIDH 0x14
#define MCP_RXF4SIDL 0x15
#define MCP_RXF4EID8 0x16
#define MCP_RXF4EID0 0x17

#define MCP_RXF5SIDH 0x18
#define MCP_RXF5SIDL 0x19
#define MCP_RXF5EID8 0x1a
#define MCP_RXF5EID0 0x1b

#define MCP_RXM0SIDH 0x20
#define MCP_RXM0SIDL 0x21
#define MCP_RXM0EID8 0x22
#define MCP_RXM0EID0 0x23

#define MCP_RXM1SIDH 0x24
#define MCP_RXM1SIDL 0x25
#define MCP_RXM1EID8 0x26
#define MCP_RXM1EID0 0x27

#define MCP_TXB0SIDH 0x31
#define MCP_TXB0SIDL 0x32
#define MCP_TXB0EID8 0x33
#define MCP_TXB0EID0 0x34
#define MCP_TXB0DLC 0x35
#define MCP_TXB0D0 0x36
#define MCP_TXB0D1 0x37
#define MCP_TXB0D2 0x38
#define MCP_TXB0D3 0x39
#define MCP_TXB0D4 0x3a
#define MCP_TXB0D5 0x3b
#define MCP_TXB0D6 0x3c
#define MCP_TXB0D7 0x3d

#define MCP_TXB1SIDH 0x41
#define MCP_TXB1SIDL 0x42
#define MCP_TXB1EID8 0x43
#define MCP_TXB1EID0 0x44
#define MCP_TXB1DLC 0x45
#define MCP_TXB1D0 0x46
#define MCP_TXB1D1 0x47
#define MCP_TXB1D2 0x48
#define MCP_TXB1D3 0x49
#define MCP_TXB1D4 0x4a
#define MCP_TXB1D5 0x4b
#define MCP_TXB1D6 0x4c
#define MCP_TXB1D7 0x4d

#define MCP_TXB2SIDH 0x51
#define MCP_TXB2SIDL 0x52
#define MCP_TXB2EID8 0x53
#define MCP_TXB2EID0 0x54
#define MCP_TXB2DLC 0x55
#define MCP_TXB2D0 0x56
#define MCP_TXB2D1 0x57
#define MCP_TXB2D2 0x58
#define MCP_TXB2D3 0x59
#define MCP_TXB2D4 0x5a
#define MCP_TXB2D5 0x5b
#define MCP_TXB2D6 0x5c
#define MCP_TXB2D7 0x5d

#define MCP_RXB0SIDH 0x61
#define MCP_RXB0SIDL 0x62
#define MCP_RXB0EID8 0x63
#define MCP_RXB0EID0 0x64
#define MCP_RXB0DLC 0x65
#define MCP_RXB0D0 0x66
#define MCP_RXB0D1 0x67
#define MCP_RXB0D2 0x68
#define MCP_RXB0D3 0x69
#define MCP_RXB0D4 0x6a
#define MCP_RXB0D5 0x6b
#define MCP_RXB0D6 0x6c
#define MCP_RXB0D7 0x6d

#define MCP_RXB1SIDH 0x71
#define MCP_RXB1SIDL 0x72
#define MCP_RXB1EID8 0x73
#define MCP_RXB1EID0 0x74
#define MCP_RXB1DLC 0x75
#define MCP_RXB1D0 0x76
#define MCP_RXB1D1 0x77
#define MCP_RXB1D2 0x78
#define MCP_RXB1D3 0x79
#define MCP_RXB1D4 0x7a
#define MCP_RXB1D5 0x7b
#define MCP_RXB1D6 0x7c
#define MCP_RXB1D7 0x7d

// BIT DEFINITIONS

// TXBnSIDL
#define MCP_EXIDE 3

// TXBnDLC
#define MCP_RTR 6

// BFPCTRL
#define MCP_B0BFM 0
#define MCP_B1BFM 1
#define MCP_B0BFE 2
#define MCP_B1BFE 3
#define MCP_B0BFS 4
#define MCP_B1BFS 5

// TXRTSCTRL
#define MCP_B0RTSM 0
#define MCP_B1RTSM 1
#define MCP_B2RTSM 2
#define MCP_B0RTS 3
#define MCP_B1RTS 4
#define MCP_B2RTS 5

// CANSTAT
#define MCP_ICOD0 1
#define MCP_ICOD1 2
#define MCP_ICOD2 3
#define MCP_OPMOD0 5
#define MCP_OPMOD1 6
#define MCP_OPMOD2 7

// CANCTRL
#define MCP_CLKPRE0 0
#define MCP_CLKPRE1 1
#define MCP_CLKEN 2
#define MCP_OSM 3
#define MCP_ABAT 4
#define MCP_REQOP0 5
#define MCP_REQOP1 6
#define MCP_REQOP2 7

// CNF1
#define MCP_BRP0 0
#define MCP_BRP1 1
#define MCP_BRP2 2
#define MCP_BRP3 3
#define MCP_BRP4 4
#define MCP_BRP5 5
#define MCP_SJW0 6
#define MCP_SJW1 7

// CNF2
#define MCP_PRSEG0 0
#define MCP_PRSEG1 1
#define MCP_PRSEG2 2
#define MCP_PHSEG10 3
#define MCP_PHSEG11 4
#define MCP_PHSEG12 5
#define MCP_SAM 6
#define MCP_BTLMODE 7

// CNF3
#define MCP_PHSEG20 0
#define MCP_PHSEG21 1
#define MCP_PHSEG22 2
#define MCP_WAKFIL 6
#define MCP_SOF 7

// CANINTE
#define MCP_RX0IE 0
#define MCP_RX1IE 1
#define MCP_TX0IE 2
#define MCP_TX1IE 3
#define MCP_TX2IE 4
#define MCP_ERRIE 5
#define MCP_WAKIE 6
#define MCP_MERRE 7

// CANINTF
#define MCP_RX0IF 0
#define MCP_RX1IF 1
#define MCP_TX0IF 2
#define MCP_TX1IF 3
#define MCP_TX2IF 4
#define MCP_ERRIF 5
#define MCP_WAKIF 6
#define MCP_MERRF 7

// EFLG
#define MCP_EWARN 0
#define MCP_RXWAR 1
#define MCP_TXWAR 2
#define MCP_RXEP 3
#define MCP_TXEP 4
#define MCP_TXBO 5
#define MCP_RX0OVR 6
#define MCP_RX1OVR 7

// TXBnCTRL
#define MCP_TXP0 0
#define MCP_TXP1 1
#define MCP_TXREQ 3
#define MCP_TXERR 4
#define MCP_MLOA 5
#define MCP_ABTF 6

// RXB0CTRL
#define MCP_FILHIT 0
#define MCP_BUKT1 1
#define MCP_BUKT 2
#define MCP_RXRTR 3
#define MCP_RXM0 5
#define MCP_RXM1 6

// RXB1CTRL
#define MCP_FILHIT0 0
#define MCP_FILHIT1 1
#define MCP_FILHIT2 2
#define MCP_RXRTR 3
#define MCP_RXM0 5
#define MCP_RXM1 6

/*
 * CAN BIT Rate Timing
 *
 * CAN bitrates calculated using: 
 *   http://www.kvaser.com/en/support/bit-timing-calculator.html
 */

// CAN Bitrate 20 kbps with 16MHz clock
// BRP = x
// T1 = 15
// T2 = 5
#define MCP_16MHZ_20kBPS_CFG1 0x13
#define MCP_16MHZ_20kBPS_CFG2 0xb6
#define MCP_16MHZ_20kBPS_CFG3 0x04

// CAN Bitrate 20 kbps with 8MHz clock
// BRP = x
// T1 = 15
// T2 = 5
#define MCP_8MHZ_20kBPS_CFG1 0x09
#define MCP_8MHZ_20kBPS_CFG2 0xb6
#define MCP_8MHZ_20kBPS_CFG3 0x04

// CAN Bitrate 125 kbps with 16MHz clock
// BRP = x
// T1 = 12
// T2 = 4
#define MCP_16MHZ_125kBPS_CFG1 0x03
#define MCP_16MHZ_125kBPS_CFG2 0xac
#define MCP_16MHZ_125kBPS_CFG3 0x03

// CAN Bitrate 125 kbps with 8MHz clock
// BRP = x
// T1 = 12
// T2 = 4
#define MCP_8MHZ_125kBPS_CFG1 0x01
#define MCP_8MHZ_125kBPS_CFG2 0xac
#define MCP_8MHZ_125kBPS_CFG3 0x03

// CAN Bitrate 250 kbps with 16MHz clock
// BRP = 4
// T1 = 12
// T2 = 4
#define MCP_16MHZ_250kBPS_CFG1 0x01
#define MCP_16MHZ_250kBPS_CFG2 0xac
#define MCP_16MHZ_250kBPS_CFG3 0x03

// CAN Bitrate 250 kbps with 8MHz clock
// BRP = x
// T1 = 12
// T2 = 4
#define MCP_8MHZ_250kBPS_CFG1 0x00
#define MCP_8MHZ_250kBPS_CFG2 0xac
#define MCP_8MHZ_250kBPS_CFG3 0x03

// CAN Bitrate 500 kbps with 16MHz clock
// BRP = x
// T1 = 12
// T2 = 4
#define MCP_16MHZ_500kBPS_CFG1 0x00
#define MCP_16MHZ_500kBPS_CFG2 0xac
#define MCP_16MHZ_500kBPS_CFG3 0x03

// CAN Bitrate 500 kbps with 8MHz clock
// BRP = x
// T1 = 6
// T2 = 2
#define MCP_8MHZ_500kBPS_CFG1 0x00
#define MCP_8MHZ_500kBPS_CFG2 0x91
#define MCP_8MHZ_500kBPS_CFG3 0x01

// CAN Bitrate 1 Mbps with 16Mhz clock
// BRP = x
// T1 = 6
// T2 = 2
#define MCP_16MHZ_1000kBPS_CFG1 0x00
#define MCP_16MHZ_1000kBPS_CFG2 0x91
#define MCP_16MHZ_1000kBPS_CFG3 0x01

// CAN Bitrate 1 Mbps with 8Mhz clock (not possible)
// BRP = x
// T1 = 6
// T2 = 2
//#define MCP_8MHZ_1000kBPS_CFG1 0x00
//#define MCP_8MHZ_1000kBPS_CFG2 0x91
//#define MCP_8MHZ_1000kBPS_CFG3 0x01

#endif
