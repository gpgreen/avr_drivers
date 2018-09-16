#ifndef BMP085_H_
#define BMP085_H_

#include "defs.h"

/*-----------------------------------------------------------------------*/

/* define if we want uart debugging output */
//#define BMP085DEBUG 1

// return error codes
#define BMP085_OK       (0)
#define BMP085_FAIL     (1)

/*-----------------------------------------------------------------------*/

// define the I2C address for this device
#define BMP085_ADDRESS (0x77 << 1)

/*-----------------------------------------------------------------------*/

// enumeration to choose which operating mode to use
enum BMP_OP_MODE {
	ULTRA_LOW_POWER = 0,
	STANDARD,
	HI_RES, 
	ULTRA_HI_RES
};

/*-----------------------------------------------------------------------*/

// enumeration to track completions of sensor reading
enum bmp_sensor_reading {
	NONE = 0,
	TEMPERATURE, 
	PRESSURE
};

/*-----------------------------------------------------------------------*/

// enumeration to track which async method is in progress
enum bmp_async_mode {
	NOOP = 0,
	TEMP, 
	PRESS
};

/*-----------------------------------------------------------------------*/

// define an interrupt vector for software interrupt
#if 0
#ifndef BMP085_SW_INT_VECT
#error "This driver requires a BMP085_SW_INT_VECT define to specify interrupt vector"
#endif
#endif

/* struct for bmp085 device specific data */
struct bmp085_priv {
	// set to 1 if interrupt happened, 0 if not
	volatile uint8_t flag;
	// interrupt control register, controlling edge detection
	volatile uint8_t * int_dir_reg;
	// mask for interrupt control register for edge detection
	uint8_t int_dir_mask;
	// interrupt control register, enable/disable
	volatile uint8_t * int_en_reg;
	// mask for interrupt control register
	uint8_t int_en_mask;
	// GPIO port interrupt is on
	volatile uint8_t * port;
	// GPIO port direction control register
	volatile uint8_t * ddr_port;
	// GPIO pin mask
	uint8_t port_pin;
};

// data structure to hold readings from device
struct bmp085_dev_t
{
	// the device number (because we have more than one)
	uint8_t num;
	
    // record completion of sensor reading
    enum bmp_sensor_reading status;
    
	// which async mode are we in..
	enum bmp_async_mode op;
	// do we have a pending op
	enum bmp_async_mode pending_op;
	
    // bmp085 specific data struct
	struct bmp085_priv dev_priv;

    // the temp in degrees K
    float temp;
    // the pressure in kPa
    float press;

	// raw temperature
	uint16_t rt;
	
	// raw pressure
	uint32_t rp;
	
	// chip id
	uint8_t chip_id;
	
	// version info
	uint8_t ml_version;
	uint8_t al_version;
	
	// param b5
	int32_t param_b5;
	
	// oversampling
	uint8_t oversampling_setting;
	
	// calibration data from prom
	int16_t ac1;
	int16_t ac2;
	int16_t ac3;
	uint16_t ac4;
	uint16_t ac5;
	uint16_t ac6;
	int16_t b1;
	int16_t b2;
	int16_t mb;
	int16_t mc;
	int16_t md;
};

/*-----------------------------------------------------------------------*/

extern int bmp085_init(enum BMP_OP_MODE omode, struct bmp085_dev_t* device);

extern int bmp085_self_test(struct bmp085_dev_t* device);

extern void bmp085_read_data(struct bmp085_dev_t* device);

extern void bmp085_read_pressure(struct bmp085_dev_t* device);

extern void bmp085_read_temperature(struct bmp085_dev_t* device);

extern void bmp085_get_temperature(struct bmp085_dev_t* p_bmp085);

extern void bmp085_get_pressure(struct bmp085_dev_t* p_bmp085);

/* async functions, start for initiating data sample.
 * call check_reading to see if a sample has been retrieved
 * call clear_reading after sample has beenconverted with get_temperature or get_pressure
 * the handle_interrupt function to be called in EOC pin interrupt handler
 * 
 * the int functions:
 * return: BMP085_OK or BMP085_FAIL
 */
extern enum bmp_sensor_reading bmp085_async_check_reading(struct bmp085_dev_t* device);
extern void bmp085_async_clear_reading(struct bmp085_dev_t* device);
/* following functions will talk to chips, so any device selector must be called before these */
extern int bmp085_async_start_temperature(struct bmp085_dev_t* device);
extern int bmp085_async_start_pressure(struct bmp085_dev_t* device);
extern int bmp085_async_handle_interrupt(struct bmp085_dev_t* device);

// extern void bmp085_check_code(void);

/*-----------------------------------------------------------------------*/

#endif
