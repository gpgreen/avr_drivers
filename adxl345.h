#ifndef ADXL345_H_
#define ADXL345_H_

#include <inttypes.h>

// define to debug device to uart
// #define ADXL345DEBUG (1)

// ----------------------
// driver struct
struct adxl345_device 
{
	// integer array which maps which axis of adxl345 is which axis
	// desired, ie no mapping axis 0 would be x, axis 1 would be y
	// so to swap x, y axis, this array would be: 1,0,2
	int8_t axis_map[3];
	// multiplier to raw data to correct sign of axis data
	// this is applied AFTER the axis map
	int8_t sign_map[3];
};

// initialize the device
extern void adxl345_init(struct adxl345_device* device);

// return 0 if ok, 1 if not
extern uint8_t adxl345_self_test(void);

// read the accelerations, put them in global variables
extern void adxl345_read_accel(void);

// read from the device and average to find the current biases
extern void adxl345_read_bias(void);

// store the current bias and gravity in eeprom
extern void adxl345_store_eeprom(void);

// active the internal self test function and see results
extern void adxl345_internal_self_test(void);

// get the corrected acceleration values per axis
float adxl345_accel(int axis);

#endif  // ADXL345_H_
