#ifndef HMC5843_H_
#define HMC5843_H_

#include <stdint.h>

////////////////////////////////////////////////////////////////////////////

// uncomment or define to get uart debugging output
//#define HMC5843DEBUG (1)

////////////////////////////////////////////////////////////////////////////

// methods

// the initialization fn
extern void hmc5843_init(void);

// return 0 if ok, 1 if not
extern uint8_t hmc5843_self_test(void);

// return 0 if ok, -1 if twi comm failure
extern uint8_t hmc5843_read_data(void);

// get the raw data
extern int16_t raw_mag_data(int axis);

////////////////////////////////////////////////////////////////////////////

#endif  // HMC5843_H_
