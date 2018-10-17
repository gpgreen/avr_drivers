#include "defs.h"
#include <stdio.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
#include "hmc5883l.h"
#include "hmc5883l_def.h"
#include "globals.h"
#include <i2cmaster.h>

////////////////////////////////////////////////////////////////////////////

// conversion from LSB read by the device to Gauss
// we are using the default Sensor Field range, which is +-1.3 Ga
// if we were to change the gain, then we need to change this
// constant also, this is gain 0x1 in configuration b register
const float k_gain = 1090.0;

// write bytes to register on the HMC5883L
static int hmc5883l_write(uint8_t reg, uint8_t* buf, int bufcount)
{
#ifdef HMCDEBUG
	printf_P(PSTR("hmc5883l_write: %x, %d\n"), reg, bufcount);
#endif
	
	i2c_start_wait(HMC5883L_ADDRESS+I2C_WRITE);
	
	if(i2c_write(reg))
		return HMC5883L_COMFAIL;

	for (int i=0; i<bufcount; ++i)
		if (i2c_write(buf[i]))
			return HMC5883L_COMFAIL;

	i2c_stop();
	
#ifdef HMCDEBUG
	printf_P(PSTR("data written -"));
	for (int i=0; i<bufcount; ++i)
		printf_P(PSTR(" byte%d:%x"), i, buf[i]);
	puts_P(PSTR("\nhmc5883l_write complete."));
#endif

	return HMC5883L_OK;
}

// read bytes from register on the HMC5883L
static int hmc5883l_read(uint8_t reg, uint8_t* buf, int bufcount)
{
#ifdef HMCDEBUG
	printf_P(PSTR("hmc5883l_read: %x, %d\n"), reg, bufcount);
#endif

	i2c_start_wait(HMC5883L_ADDRESS+I2C_WRITE);

	if (i2c_write(reg))
		return HMC5883L_COMFAIL;

	if (i2c_rep_start(HMC5883L_ADDRESS+I2C_READ))
		return HMC5883L_COMFAIL;

	for (int i=0; i<bufcount-1; ++i)
		buf[i] = i2c_readAck();
	buf[bufcount-1] = i2c_readNak();
	
	i2c_stop();

#ifdef HMCDEBUG
	printf_P(PSTR("data read -"));
	for (int i=0; i<bufcount; ++i)
		printf_P(PSTR(" byte%d:%x"), i, buf[i]);
	puts_P(PSTR("\nhmc5883l_read complete."));
#endif

	return HMC5883L_OK;
}

uint8_t hmc5883l_init(void)
{
    uint8_t buf = HMC_MODE_CONTINUOUS;

#ifdef HMCDEBUG
	puts_P(PSTR("HMC5883L initializing."));
#endif
	
	/* nominal 5ms power-up time */
	_delay_ms(5);
	
    /* set measurement mode to continuous */
    if (hmc5883l_write(HMC_MODE, &buf, 1))
        return HMC5883L_FAIL;
    _delay_ms(100);

	/* the device will be collecting at 10hz, default */
#ifdef HMCDEBUG
	puts_P(PSTR("HMC5883L done."));
#endif
	return HMC5883L_OK;
}

uint8_t hmc5883l_self_test(void)
{
	uint8_t buf[3];
	if (hmc5883l_read(HMC_IDENTA, buf, 3))
		return HMC5883L_FAIL;
	// if correct, return 0
	return !(buf[0] == HMC_IDENTA_VAL
             && buf[1] == HMC_IDENTB_VAL
             && buf[2] == HMC_IDENTC_VAL);
}

uint8_t hmc5883l_read_data(struct hmc5883l_dev_t* device)
{
    uint8_t buf[6];
    if (hmc5883l_read(HMC_DATAX0, buf, 6))
        return HMC5883L_COMFAIL;

	// the data registers are in X,Z,Y order
    device->raw_mag[0] = (buf[0] << 8) | buf[1];    // X axis
    device->raw_mag[2] = (buf[2] << 8) | buf[3];    // Z axis
    device->raw_mag[1] = (buf[4] << 8) | buf[5];    // Y axis

	// TODO: need to check for saturation and flag an error if it happens
    return HMC5883L_OK;
}

float hmc5883l_raw_mag_data(struct hmc5883l_dev_t* device, int axis)
{
    return device->sensor_sign[axis]
		* (float)(device->raw_mag[axis]) / k_gain;
}

/*
 * device has a built in scale test. An artificial magnetic field
 * is applied, and the resulting measurement subtracted from the
 * measurement before the artificial field. The result is the
 * measurement of the (fixed) artificial field. For the gain
 * chosen below (gain=3), the artificial field should measure
 * +766 LSB in X and Y, and +713 in Z. Any deviation from those
 * values can be applied to the following measurements. For example
 * the x results in 750, scale the following measurements by
 * 766/750. Do the same for all three axis
 */
uint8_t hmc5883l_scale_test(struct hmc5883l_dev_t *dev,
                            struct hmc5883l_scale_test_t *tst)
{
    uint8_t saved_cfga, saved_cfgb, saved_mode;
    uint8_t buf[6];

    // save the current state
    if (hmc5883l_read(HMC_MODE, &saved_mode, 1))
        return HMC5883L_COMFAIL;
    if (hmc5883l_read(HMC_CFGA, &saved_cfga, 1))
        return HMC5883L_COMFAIL;
    if (hmc5883l_read(HMC_CFGB, &saved_cfgb, 1))
        return HMC5883L_COMFAIL;
    
    // change to positive bias mode
    buf[0] = (HMC_CFGA_MA_8 << HMC_CFGA_MA_SHIFT)
        | HMC_CFGA_MM_POS_BIAS;
	if (hmc5883l_write(HMC_CFGA, buf, 1))
		return HMC5883L_COMFAIL;
    // change the gain, this test adds about 1.1gauss
    buf[0] = 0x60;              // 2.5 Ga
    if (hmc5883l_write(HMC_CFGB, buf, 1))
        return HMC5883L_COMFAIL;
    // place in single measurement mode
    buf[0] = HMC_MODE_SINGLE;
    if (hmc5883l_write(HMC_MODE, buf, 1))
        return HMC5883L_COMFAIL;
    // wait for result, (mode register goes to idle)
    while (1) {
        if (hmc5883l_read(HMC_MODE, buf, 1))
            return HMC5883L_COMFAIL;
        if ((buf[0] & HMC_MODE_MASK) > HMC_MODE_SINGLE)
            break;
        _delay_ms(1);
    }
    // read the self test data
    if (hmc5883l_read(HMC_DATAX0, buf, 6))
        return HMC5883L_COMFAIL;
	// the data registers are in X,Z,Y order
    tst->raw_mag[0] = (buf[0] << 8) | buf[1];    // X axis
    tst->raw_mag[2] = (buf[2] << 8) | buf[3];    // Z axis
    tst->raw_mag[1] = (buf[4] << 8) | buf[5];    // Y axis

    // return the mode and config registers back to original state
    if (hmc5883l_write(HMC_CFGA, &saved_cfga, 1))
        return HMC5883L_COMFAIL;
    if (hmc5883l_write(HMC_CFGB, &saved_cfgb, 1))
        return HMC5883L_COMFAIL;
    if (hmc5883l_write(HMC_MODE, &saved_mode, 1))
        return HMC5883L_COMFAIL;

    return HMC5883L_OK;
}

