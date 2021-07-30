#ifndef L3G4200D_H_
#define L3G4200D_H_

#include <stdint.h>

// define to 1 if debugging
/* #define L3G4200DDEBUG   (1) */

// return error codes
#define L3G4200D_OK      (0)
#define L3G4200D_FAIL    (1)
#define L3G4200D_COMFAIL (2)

////////////////////////////////////////////////////////////////////////////

typedef struct 
{
	uint8_t temp;
	int16_t raw_rate[3];
	int sensor_sign[3];
} l3g4200d_dev_t;

////////////////////////////////////////////////////////////////////////////
// methods

// the initialization fn
extern void l3g4200d_init(void);

// return L3G4200D_OK, or L3G4200D_FAIL
extern uint8_t l3g4200d_self_test(void);

// return L3G4200D_OK, L3G4200D_COMFAIL if spi communication failure
extern uint8_t l3g4200d_read_data(l3g4200d_dev_t* device);

// get the raw data
extern float l3g4200d_raw_data(l3g4200d_dev_t* device, int axis);

#endif  // L3G4200D_H_
