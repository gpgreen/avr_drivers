#ifndef HMC5883L_DEF_H_
#define HMC5883L_DEF_H_

/* registers */
#define HMC_CFGA           0x00
#define HMC_CFGB           0x01
#define HMC_MODE           0x02
#define HMC_DATAX0         0x03
#define HMC_DATAX1         0x04
#define HMC_DATAY0         0x05
#define HMC_DATAY1         0x06
#define HMC_DATAZ0         0x07
#define HMC_DATAZ1         0x08
#define HMC_STATUS         0x09
#define HMC_IDENTA         0x0a
#define HMC_IDENTB         0x0b
#define HMC_IDENTC         0x0c

/* Configuration Register A bit flags */
#define HMC_CRA_MA1        6
#define HMC_CRA_MA0        5
#define HMC_CRA_DO2        4
#define HMC_CRA_DO1        3
#define HMC_CRA_DO0        2
#define HMC_CRA_MS1        1
#define HMC_CRA_MS0        0

/* CFGA measurement sample average */
#define HMC_CFGA_MA_MASK        0x6
#define HMC_CFGA_MA_SHIFT       5
#define HMC_CFGA_MA_1           0x0
#define HMC_CFGA_MA_2           0x1
#define HMC_CFGA_MA_4           0x2
#define HMC_CFGA_MA_8           0x3  // default

/* CFGA data output rate */
#define HMC_CFGA_DOR_MASK       0x16
#define HMC_CFGA_DOR_SHIFT      2
#define HMC_CFGA_DOR_0_75       0x0
#define HMC_CFGA_DOR_1_5        0x1
#define HMC_CFGA_DOR_3          0x2
#define HMC_CFGA_DOR_7_5        0x3
#define HMC_CFGA_DOR_15         0x4  // default
#define HMC_CFGA_DOR_30         0x5
#define HMC_CFGA_DOR_75         0x6

/* CFGA measurement mode */
#define HMC_CFGA_MM_MASK        0x3
#define HMC_CFGA_MM_SHIFT       0
#define HMC_CFGA_MM_NORMAL      0x0  // default
#define HMC_CFGA_MM_POS_BIAS    0x1
#define HMC_CFGA_MM_NEG_BIAS    0x2

/* CFGB bits */
#define HMC_CFGB_GAIN_MASK      0x7
#define HMC_CFGB_GAIN_SHIFT     5

/* Mode bits */
#define HMC_MODE_MASK            0x3
#define HMC_MODE_SHIFT           0
#define HMC_MODE_CONTINUOUS      0x00
#define HMC_MODE_SINGLE          0x1  // default
#define HMC_MODE_IDLE1           0x2
#define HMC_MODE_IDLE2           0x3

/* Status bits */
#define HMC_STATUS_LOCK    1
#define HMC_STATUS_RDY     0

/* ident values */
#define HMC_IDENTA_VAL     0x48
#define HMC_IDENTB_VAL     0x34
#define HMC_IDENTC_VAL     0x33

#endif  // HMC5883L_DEF_H_
