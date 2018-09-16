#ifndef HMC5883L_H_
#define HMC5883L_H_

#include <inttypes.h>

// define to 1 if debugging
/* #define HMCDEBUG   (1) */

// return error codes
#define HMC5883L_OK      (0)
#define HMC5883L_FAIL    (1)
#define HMC5883L_COMFAIL (2)

// i2c address
#define HMC5883L_ADDRESS (0x3c)

////////////////////////////////////////////////////////////////////////////

struct hmc5883l_dev_t
{
	int16_t raw_mag[3];
	int sensor_sign[3];
};

struct hmc5883l_scale_test_t 
{
    int16_t raw_mag[3];
};

////////////////////////////////////////////////////////////////////////////
// methods

// the initialization fn
extern uint8_t hmc5883l_init(void);

// return HMC5883L_OK, or HMC5883L_FAIL
extern uint8_t hmc5883l_self_test(void);

// return HMC5883L_OK, HMC5883L_COMFAIL if twi communication failure
extern uint8_t hmc5883l_read_data(struct hmc5883l_dev_t* device);

// get the raw data
extern float hmc5883l_raw_mag_data(struct hmc5883l_dev_t* device, int axis);

// execute the scale test
// return HMC5883L_OK, HMC5883L_COMFAIL if twi communication failure
extern uint8_t hmc5883l_scale_test(struct hmc5883l_dev_t *dev,
                                   struct hmc5883l_scale_test_t *tst);

#endif  // HMC5883L_H_
