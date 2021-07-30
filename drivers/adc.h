#ifndef ADC_H_
#define ADC_H_

#include <stdint.h>

// uncomment to allow printing to serial port for debugging
/* #define ADCDEBUG 1 */

//-----------------------
// structure to hold axis mapping
struct gyro_device 
{
	// integer array which maps which axis of gyros is which axis
	// desired, ie no mapping axis 0 would be x, axis 1 would be y
	// so to swap x, y axis, this array would be: 1,0,2
	int8_t axis_map[3];
	// multiplier to raw data to correct sign of axis data
	// this is applied AFTER the axis map
	int8_t sign_map[3];
};

extern void adc_init(struct gyro_device* device);
extern void adc_read(void);
	
extern float get_gyro_data(int axis);
extern int16_t get_gyro_bias(int axis);

extern void gyro_self_test(void);
extern void gyro_set_bias(void);
	
#endif  // ADC_H_
