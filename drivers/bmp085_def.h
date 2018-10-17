#ifndef BMP085_DEF_H_
#define BMP085_DEF_H_

#ifdef BMP085DEBUG

#  ifdef __AVR
#    include <avr/pgmspace.h>

#    define PDEBUG(MSG) (printf_P(PSTR("%s" MSG "\n"), k_name))
#    define PDEBUG2(MSG, ...) (printf_P(PSTR("%s" MSG "\n"), k_name, __VA_ARGS__))

#  else

#    define PDEBUG(MSG) (printf("%s" MSG "\n", k_name))
#    define PDEBUG2(MSG, ...) (printf("%s" MSG "\n", k_name, __VA_ARGS__))

#  endif

#else

#  define PDEBUG(MSG) /* nothing */
#  define PDEBUG2(MSG, ...) /* nothing */

#endif

// chip type constants
#define BMP085_CHIP_ID			0x55
#define BOSCH_PRESSURE_BMP085	85

// calibration parameters
#define SMD500_PARAM_MG      3038        //calibration parameter
#define SMD500_PARAM_MH     -7357        //calibration parameter
#define SMD500_PARAM_MI      3791        //calibration parameter

// oversampling_setting
#define BMP_OP_ULTRA_LO_PWR_MODE 0
#define BMP_OP_STANDARD_MODE 1
#define BMP_OP_HI_RES_MODE 2
#define BMP_OP_ULTRA_HI_RES_MODE 3

// registers
#define BMP085_PROM_START__ADDR 0xaa
#define BMP085_PROM_DATA__LEN		  22

#define BMP085_CHIP_ID_REG			0xD0
#define BMP085_VERSION_REG			0xD1

#define BMP085_CTRL_MEAS_REG		0xF4
#define BMP085_ADC_OUT_MSB_REG		0xF6
#define BMP085_ADC_OUT_LSB_REG		0xF7

#define BMP085_SOFT_RESET_REG		0xE0

#define BMP085_T_MEASURE        0x2E				// temperature measurent 
#define BMP085_P_MEASURE        0x34				// pressure measurement

#define BMP085_TEMP_CONVERSION_TIME  5				// TO be spec'd by GL or SB

/*
 * bit slice positions in registers
 *
 */
#define BMP085_CHIP_ID__POS		0
#define BMP085_CHIP_ID__MSK		0xFF
#define BMP085_CHIP_ID__LEN		8
#define BMP085_CHIP_ID__REG		BMP085_CHIP_ID_REG


#define BMP085_ML_VERSION__POS		0
#define BMP085_ML_VERSION__LEN		4
#define BMP085_ML_VERSION__MSK		0x0F
#define BMP085_ML_VERSION__REG		BMP085_VERSION_REG



#define BMP085_AL_VERSION__POS  	4
#define BMP085_AL_VERSION__LEN  	4
#define BMP085_AL_VERSION__MSK		0xF0
#define BMP085_AL_VERSION__REG		BMP085_VERSION_REG

#define BMP085_GET_BITSLICE(regvar, bitname)\
			(regvar & bitname##__MSK) >> bitname##__POS


#define BMP085_SET_BITSLICE(regvar, bitname, val)\
		  (regvar & ~bitname##__MSK) | ((val<<bitname##__POS)&bitname##__MSK)  

#endif  // BMP085_DEF_H_
