#include "defs.h"
#include <stdio.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
#include "l3g4200d.h"
#include "l3g4200d_def.h"
#include "globals.h"
#include <i2cmaster.h>

////////////////////////////////////////////////////////////////////////////

/*
 * write multiple register's on the L3G4200D
 */
static int l3g4200d_write_registers( uint8_t address, uint8_t* data, uint8_t len )
{
#ifdef L3G4200DDEBUG
	printf_P(PSTR("write_registers:%x,"), address);
	for (int i=0; i<len; ++i)
		printf_P(PSTR("%x,"), data[i]);
	printf("\n");
#endif

	// if our data len is greater than 1, we want auto increment of
	// registers
	if(len > 1)
		address |= _BV(L3G4200D_MS);
	
	i2c_start_wait(L3G4200D_SDO_GND_ADDRESS+I2C_WRITE);

	if (i2c_write(address))
		return -1;
	
	for (int i=0; i<len; ++i)
		if (i2c_write(data[i]))
			return -1;
			
	i2c_stop();
	
#ifdef L3G4200DDEBUG
	printf_P(PSTR("done write_registers\n"));
#endif

	return 0;
}

/*
 * read multiple register's on the L3G4200D
 */
static int l3g4200d_read_registers( uint8_t address, uint8_t* data, uint8_t len )
{
#ifdef L3G4200DDEBUG
	printf_P(PSTR("read_registers:%x\n"), address);
#endif

	// if our data len is greater than 1, we want auto increment of
	// registers
	if(len > 1)
		address |= _BV(L3G4200D_MS);
	
	i2c_start_wait(L3G4200D_SDO_GND_ADDRESS+I2C_WRITE);

	if (i2c_write(address))
		return -1;

	if (i2c_rep_start(L3G4200D_SDO_GND_ADDRESS+I2C_READ))
		return -1;

	for (int i=0; i<len-1; ++i)
		data[i] = i2c_readAck();
	data[len-1] = i2c_readNak();
	
	i2c_stop();
	
#ifdef L3G4200DDEBUG
	printf_P(PSTR("done read_registers:"));
	for (int i=0; i<len; ++i)
		printf_P(PSTR("%x,"), data[i]);
	printf("\n");
#endif

	return 0;
}

void l3g4200d_init(void)
{
    uint8_t buf;
    buf = 0x00;

#ifdef L3G4200DDEBUG
	puts_P(PSTR("L3G4200D initializing."));
#endif
	
	/* nominal 5ms power-up time */
	_delay_ms(5);

	/* get the WHO_AM_I register, make sure it is ok */
	if (l3g4200d_read_registers(L3G_WHOAMI, &buf, 1) || buf != 0b11010011)
		failed(7);
	
	/* ODR 100hz, BW cutoff 12.5, normal mode */
	buf = _BV(PD)|_BV(ZEN)|_BV(YEN)|_BV(XEN);
	if (l3g4200d_write_registers(L3G_CTRL_REG1, &buf, 1))
		failed(7);

	/* High Pass Filter Normal Mode, 8Hz cutoff frequency */
	buf = _BV(HPM1);
	if (l3g4200d_write_registers(L3G_CTRL_REG2, &buf, 1))
		failed(7);

	/* Interrupt setup */
	buf = 0;
	if (l3g4200d_write_registers(L3G_CTRL_REG3, &buf, 1))
		failed(7);

	/* Block Data update
	   Scale selection 500deg/s */
	buf = _BV(FS0);
	if (l3g4200d_write_registers(L3G_CTRL_REG4, &buf, 1))
		failed(7);

	/* High Pass filter */
	buf = 0;
	if (l3g4200d_write_registers(L3G_CTRL_REG5, &buf, 1))
		failed(7);

	/* FIFO control */
	buf = 0;
	if (l3g4200d_write_registers(L3G_FIFO_CTRL_REG, &buf, 1))
		failed(7);

#ifdef L3G4200DSLEEP
	/* put the device to sleep */
	buf = _BV(PD);
	if (l3g4200d_write_registers(L3G_CTRL_REG1, &buf, 1))
		failed(7);
#endif	
	
#ifdef L3G4200DDEBUG
	puts_P(PSTR("L3G4200D done."));
#endif
}

uint8_t l3g4200d_self_test(void)
{
	uint8_t buf;
	
	if (l3g4200d_read_registers(L3G_WHOAMI, &buf, 1) || buf != 0b11010011)
		failed(7);
	// if correct, return 0
	return 0;
}

uint8_t l3g4200d_read_data(l3g4200d_dev_t* device)
{
    uint8_t buf[8];

#ifdef L3G4200DSLEEP

	// wake up
	buf[0] = _BV(PD)|_BV(ZEN)|_BV(YEN)|_BV(XEN);
	if (l3g4200d_write_registers(L3G_CTRL_REG1, buf, 1))
		failed(7);
	// loop till data is available
	do {
		if(l3g4200d_read_registers(L3G_STATUS_REG, buf, 1))
			failed(7);
	} while((buf[0] & _BV(ZYXDA)) != _BV(ZYXDA));
	printf_P(PSTR("L3G4200D status:%x\n"), buf[0]);
	
#endif	

	// read temp, status, x, y, z
	if (l3g4200d_read_registers(L3G_OUT_TEMP, buf, 8))
		failed(7);
	
	// the data registers are in X,Y,Z order
	device->temp = buf[0];
    device->raw_rate[0] = (buf[3] << 8) | buf[2];    // X axis
    device->raw_rate[1] = (buf[5] << 8) | buf[4];    // Z axis
    device->raw_rate[2] = (buf[7] << 8) | buf[6];    // Y axis

#ifdef L3G4200DDEBUG
	printf_P(PSTR("L3G4200D status:%x\n"), buf[1]);

	// read FIFO status
	if(l3g4200d_read_registers(L3G_FIFO_SRC_REG, buf, 1))
		failed(7);
	printf_P(PSTR("L3G4200D fifo status:%x\n"), buf[0]);
#endif
	
#ifdef L3G4200DSLEEP
	// go back to sleep
	buf[0] = _BV(PD);
	if (l3g4200d_write_registers(L3G_CTRL_REG1, buf, 1))
		failed(7);
#endif	

	// TODO: need to check for saturation and flag an error if it happens
    return L3G4200D_OK;
}

float l3g4200d_raw_data(l3g4200d_dev_t* device, int axis)
{
    return device->sensor_sign[axis] * (float)(device->raw_rate[axis]);
}


