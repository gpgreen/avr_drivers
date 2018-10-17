#ifndef L3G4200D_DEF_H_
#define L3G4200D_DEF_H_

/* I2C address */

/* SDO grounded 1101000b */
#define L3G4200D_SDO_GND_ADDRESS   (0x68 << 1)
/* SDO to VDD */
#define L3G4200D_SDO_VDD_ADDRESS   (0x69 << 1)

/* RW bit */
#define L3G4200D_RW        7	// Write is 0, Read is 1
/* MS bit */
#define L3G4200D_MS        7	// 0 is no-inc, 1 is increment

/* registers */
#define L3G_WHOAMI         0x0f // r
#define L3G_CTRL_REG1      0x20 // rw
#define L3G_CTRL_REG2      0x21 // rw
#define L3G_CTRL_REG3      0x22 // rw
#define L3G_CTRL_REG4      0x23 // rw
#define L3G_CTRL_REG5      0x24 // rw
#define L3G_REFERENCE      0x25 // rw
#define L3G_OUT_TEMP       0x26 // r
#define L3G_STATUS_REG     0x27 // r
#define L3G_OUT_X_L        0x28 // r
#define L3G_OUT_X_H        0x29 // r
#define L3G_OUT_Y_L        0x2a // r
#define L3G_OUT_Y_H        0x2b // r
#define L3G_OUT_Z_L        0x2c // r
#define L3G_OUT_Z_H        0x2d // r
#define L3G_FIFO_CTRL_REG  0x2e // rw
#define L3G_FIFO_SRC_REG   0x2f // rw
#define L3G_INT1_CFG       0x30 // rw
#define L3G_INT1_SRC       0x31 // r
#define L3G_INT1_TSH_XH    0x32 // rw
#define L3G_INT1_TSH_XL    0x33 // rw
#define L3G_INT1_TSH_YH    0x34 // rw
#define L3G_INT1_TSH_YL    0x35 // rw
#define L3G_INT1_TSH_ZH    0x36 // rw
#define L3G_INT1_TSH_ZL    0x37 // rw
#define L3G_INT1_DURATION  0x38 // rw

/* bit positions */

/* CTRL_REG1 */
#define DR1                7
#define DR0                6
#define BW1                5
#define BW0                4
#define PD                 3
#define ZEN                2
#define YEN                1
#define XEN                0

/* CTRL_REG2 */
#define HPM1               5
#define HPM0               4
#define HPCF3              3
#define HPCF2              2
#define HPCF1              1
#define HPCF0              0

/* CTRL_REG3 */
#define I1_INT1            7
#define I1_BOOT            6
#define H_LACTIVE          5
#define PP_OD              4
#define I2_DRDY            3
#define I2_WTM             2
#define I2_ORUN            1
#define I2_EMPTY           0

/* CTRL_REG4 */
#define BDU                7
#define BLE                6
#define FS1                5
#define FS0                4
#define ST1                2
#define ST0                1
#define SIM                0

/* CTRL_REG5 */
#define BOOT               7
#define FIFO_EN            6
#define HPEN               4
#define INT1_SEL1          3
#define INT1_SEL0          2
#define OUT_SEL1           1
#define OUT_SEL0           0

/* STATUS_REG */
#define ZYXOR              7
#define ZOR                6
#define YOR                5
#define XOR                4
#define ZYXDA              3
#define ZDA                2
#define YDA                1
#define XDA                0

/* FIFO_CTRL_REG */
#define FM2                7
#define FM1                6
#define FM0                5
#define WTM4               4
#define WTM3               3
#define WTM2               2
#define WTM1               1
#define WTM0               0

/* FIFO_SRC_REG */
#define WTM                7
#define OVRN               6
#define EMPTY              5
#define FSS4               4
#define FSS3               3
#define FSS2               2
#define FSS1               1
#define FSS0               0

/* INT1_CFG */
#define ANDOR              7
#define LIR                6
#define ZHIE               5
#define ZLIE               4
#define YHIE               3
#define YLIE               2
#define XHIE               1
#define XLIE               0

/* INT1_SRC */
#define IA                 6
#define ZHSRC              5
#define ZLSRC              4
#define YHSRC              3
#define YLSRC              2
#define XHSRC              1
#define XLSRC              0

/* INT1_DURATION */
#define WAIT               7
#define D6                 6
#define D5                 5
#define D4                 4
#define D3                 3
#define D2                 2
#define D1                 1
#define D0                 0

#endif  // L3G4200D_DEF_H_
