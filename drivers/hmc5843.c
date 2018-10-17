#include "defs.h"
#include <stdio.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
#include "hmc5843.h"
#include "hmc5843_def.h"
#include "globals.h"
#include <i2cmaster.h>

////////////////////////////////////////////////////////////////////////////
// globals
static int16_t s_raw_mag[3];

// write bytes to register on the HMC5843
static int hmc5843_write(uint8_t reg, uint8_t* buf, int bufcount)
{
#ifdef HMC5843DEBUG
	printf_P(PSTR("hmc5843_write reg(%x)"), reg);
	for (int i=0; i<bufcount; ++i)
		printf_P(PSTR(",%x"), buf[i]);
	puts("");
#endif

	i2c_start_wait(HMC5843_ADDRESS+I2C_WRITE);
	
	if(i2c_write(reg))
		return -1;

	for (int i=0; i<bufcount; ++i)
		if (i2c_write(buf[i]))
			return -1;

	i2c_stop();

#ifdef HMC5843DEBUG
	puts_P(PSTR("hmc5843_write done"));
#endif
	
	return 0;
}

// read bytes from register on the HMC5843
static int hmc5843_read(uint8_t reg, uint8_t* buf, int bufcount)
{
#ifdef HMC5843DEBUG
	puts_P(PSTR("hmc5843_read start"));
#endif
	
	i2c_start_wait(HMC5843_ADDRESS+I2C_WRITE);

	if (i2c_write(reg))
		return -1;

	if (i2c_rep_start(HMC5843_ADDRESS+I2C_READ))
		return -1;

	for (int i=0; i<bufcount-1; ++i)
		buf[i] = i2c_readAck();
	buf[bufcount-1] = i2c_readNak();
	
	i2c_stop();

#ifdef HMC5843DEBUG
	printf_P(PSTR("hmc5843_read reg(%x)"), reg);
	for (int i=0; i<bufcount; ++i)
		printf_P(PSTR(",%x"), buf[i]);
	puts(" done.");
#endif

	return 0;
}

void hmc5843_init(void)
{
    uint8_t buf;
    buf = 0x00;

#ifdef HMC5843DEBUG
	puts_P(PSTR("hmc5843 initializing"));
#endif
	
	/* nominal 5ms power-up time */
	_delay_ms(5);
	
    /* set measurement mode to continuous */
    if (hmc5843_write(HMC_MODE, &buf, 1))
        failed(6);
    _delay_ms(100);

	/* the device will be collecting at 10hz, default */
#ifdef HMC5843DEBUG
	puts_P(PSTR("hmc5843 setup"));
#endif
}

uint8_t hmc5843_self_test(void)
{
	uint8_t buf[3];
	if (hmc5843_read(HMC_IDENTA, buf, 3))
		failed(7);
	// if correct, return 0
	return !(buf[0] == 0x48 && buf[1] == 0x34 && buf[2] == 0x33);
}

uint8_t hmc5843_read_data(void)
{
    uint8_t buf[6];
    if (hmc5843_read(HMC_DATAX0, buf, 6))
        return -1;
    
    s_raw_mag[0] = (buf[2] << 8) | buf[3];    // Y axis (internal sensor x axis)
    s_raw_mag[1] = (buf[0] << 8) | buf[1];    // X axis (internal sensor y axis)
    s_raw_mag[2] = (buf[4] << 8) | buf[5];    // Z axis

    return 0;
}

int16_t raw_mag_data(int axis)
{
	return s_raw_mag[axis];
}


