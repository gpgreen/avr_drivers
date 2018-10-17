#ifndef AT90CAN_DEF_H_
#define AT90CAN_DEF_H_

/*
 * CAN BIT Rate Timing
 *
 * CAN bitrates calculated using: 
 *   http://www.kvaser.com/en/support/bit-timing-calculator.html
 */

// CAN Bitrate 100 kbps with 16MHz clock
// BRP = x
// T1 = 15
// T2 = 5
#define AT90CAN_16MHZ_100kBPS_CFG1 0x12
#define AT90CAN_16MHZ_100kBPS_CFG2 0x0c
#define AT90CAN_16MHZ_100kBPS_CFG3 0x37

// CAN Bitrate 100 kbps with 8MHz clock
// BRP = x
// T1 = 15
// T2 = 5
#define AT90CAN_8MHZ_100kBPS_CFG1 0x08
#define AT90CAN_8MHZ_100kBPS_CFG2 0x0c
#define AT90CAN_8MHZ_100kBPS_CFG3 0x37

// CAN Bitrate 125 kbps with 16MHz clock
// BRP = x
// T1 = 12
// T2 = 4
#define AT90CAN_16MHZ_125kBPS_CFG1 0x0e
#define AT90CAN_16MHZ_125kBPS_CFG2 0x0c
#define AT90CAN_16MHZ_125kBPS_CFG3 0x37

// CAN Bitrate 125 kbps with 8MHz clock
// BRP = x
// T1 = 12
// T2 = 4
#define AT90CAN_8MHZ_125kBPS_CFG1 0x06
#define AT90CAN_8MHZ_125kBPS_CFG2 0x0c
#define AT90CAN_8MHZ_125kBPS_CFG3 0x37

// CAN Bitrate 250 kbps with 16MHz clock
// BRP = 4
// T1 = 12
// T2 = 4
#define AT90CAN_16MHZ_250kBPS_CFG1 0x06
#define AT90CAN_16MHZ_250kBPS_CFG2 0x0c
#define AT90CAN_16MHZ_250kBPS_CFG3 0x37

// CAN Bitrate 250 kbps with 8MHz clock
// BRP = x
// T1 = 12
// T2 = 4
#define AT90CAN_8MHZ_250kBPS_CFG1 0x02
#define AT90CAN_8MHZ_250kBPS_CFG2 0x0c
#define AT90CAN_8MHZ_250kBPS_CFG3 0x37

// CAN Bitrate 500 kbps with 16MHz clock
// BRP = x
// T1 = 12
// T2 = 4
#define AT90CAN_16MHZ_500kBPS_CFG1 0x02
#define AT90CAN_16MHZ_500kBPS_CFG2 0x0c
#define AT90CAN_16MHZ_500kBPS_CFG3 0x37

// CAN Bitrate 500 kbps with 8MHz clock
// BRP = x
// T1 = 6
// T2 = 2
#define AT90CAN_8MHZ_500kBPS_CFG1 0x00
#define AT90CAN_8MHZ_500kBPS_CFG2 0x0c
#define AT90CAN_8MHZ_500kBPS_CFG3 0x37

// CAN Bitrate 1 Mbps with 16Mhz clock
// BRP = x
// T1 = 6
// T2 = 2
#define AT90CAN_16MHZ_1000kBPS_CFG1 0x00
#define AT90CAN_16MHZ_1000kBPS_CFG2 0x0c
#define AT90CAN_16MHZ_1000kBPS_CFG3 0x37

// CAN Bitrate 1 Mbps with 8Mhz clock (not possible)
// BRP = x
// T1 = 6
// T2 = 2
//#define AT90CAN_8MHZ_1000kBPS_CFG1 0x00
//#define AT90CAN_8MHZ_1000kBPS_CFG2 0x91
//#define AT90CAN_8MHZ_1000kBPS_CFG3 0x01

// masks for use in code
#define MOB_ERROR_MASK 0x1f
#define GENERAL_ERROR_MASK 0x5f

#endif
