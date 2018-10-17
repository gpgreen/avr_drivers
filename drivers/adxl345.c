#include "defs.h"

#ifdef ADXL345DEBUG
#include <stdio.h>
#include <avr/pgmspace.h>
#endif

#include <avr/io.h>
#include <avr/eeprom.h>
#include <util/delay.h>
#include "adxl345.h"
#include "adxl345_def.h"
#include "globals.h"
#include <i2cmaster.h>

// eeprom storage structure
struct adxl345_eeprom_t
{
	int8_t acc_bias[3];
	uint8_t gravity;
};

// initialized structure for eeprom
struct adxl345_eeprom_t s_ee EEMEM = {
	{0, 0, 0},
	248
};

// this will be populated from eeprom during device initialization
struct adxl345_eeprom_t g_ee;

// raw accelerometer data
uint16_t raw_acc[3];

// accelerometer data corrected for bias
int16_t corrected_acc[3];

// accelerometer biases
int8_t acc_bias[3];

// static pointer to device structure
static struct adxl345_device* s_device;

// write bytes to register on the ADXL345
static int adxl345_write(uint8_t reg, uint8_t* buf, int bufcount)
{
#ifdef ADXL345DEBUG
	printf_P(PSTR("write_registers:%x,"), reg);
	for (int i=0; i<bufcount; ++i)
		printf_P(PSTR("%x,"), buf[i]);
	printf("\n");
#endif

	i2c_start_wait(ADXL345_ADDRESS+I2C_WRITE);
	
	if(i2c_write(reg))
		return -1;

	for (int i=0; i<bufcount; ++i)
		if (i2c_write(buf[i]))
			return -1;

	i2c_stop();

#ifdef ADXL345DEBUG
	printf_P(PSTR("done write_registers\n"));
#endif

	return 0;
}

// read bytes from register on the ADXL345
static int adxl345_read(uint8_t reg, uint8_t* buf, int bufcount)
{
#ifdef ADXL345DEBUG
	printf_P(PSTR("read_registers:%x\n"), reg);
#endif

	i2c_start_wait(ADXL345_ADDRESS+I2C_WRITE);

	if (i2c_write(reg))
		return -1;

	if (i2c_rep_start(ADXL345_ADDRESS+I2C_READ))
		return -1;

	for (int i=0; i<bufcount-1; ++i)
		buf[i] = i2c_readAck();
	buf[bufcount-1] = i2c_readNak();
	
	i2c_stop();

#ifdef ADXL345DEBUG
	printf_P(PSTR("done read_registers:"));
	for (int i=0; i<bufcount; ++i)
		printf_P(PSTR("%x,"), buf[i]);
	printf("\n");
#endif

	return 0;
}

// initialize the device
void adxl345_init(struct adxl345_device* device)
{
    uint8_t buf;
    buf = 0x08;

	// record address of device structure
	s_device = device;
	
#ifdef ADXL345DEBUG
	puts_P(PSTR("adxl345 initializing"));

	// print out self-test
	printf_P(PSTR("adxl345 axis map: %d,%d,%d\n"),
			 s_device->axis_map[0],s_device->axis_map[1],
			 s_device->axis_map[2]);
	printf_P(PSTR("adxl345 sign map: %d,%d,%d\n"),
			 s_device->sign_map[0],s_device->sign_map[1],
			 s_device->sign_map[2]);

#endif
	
	/* set measurement mode */
	if (adxl345_write(ADXL_POWER_CTL, &buf, 1))
		failed(1);
	
	/* read back measurement mode */
	if (adxl345_read(ADXL_POWER_CTL, &buf, 1))
		failed(1);
	if (buf != 0x08)
		failed(1);
	
    /* set full resolution, 8g range */
    buf = 0x0a;
	if (adxl345_write(ADXL_DATA_FORMAT, &buf, 1))
		failed(2);
	
	/* read back resolution */
	if (adxl345_read(ADXL_DATA_FORMAT, &buf, 1))
		failed(2);
	if (buf != 0x0a)
		failed(2);
	 
    /* set the output rate to 100 Hz, bandwidth 50 Hz */
    buf = 0x0a;
	if (adxl345_write(ADXL_BW_RATE, &buf, 1))
		failed(3);
	
	/* read back rate */
	if (adxl345_read(ADXL_BW_RATE, &buf, 1))
		failed(3);
	if (buf != 0x0a)
		failed(3);
	 
	/* read the offsets from eeprom, set in ram */
	eeprom_read_block(&g_ee, &s_ee, sizeof(struct adxl345_eeprom_t));

	/* set the biases from eeprom */
	acc_bias[0] = g_ee.acc_bias[0];
	acc_bias[1] = g_ee.acc_bias[1];
	acc_bias[2] = g_ee.acc_bias[2] - g_ee.gravity;
	
#ifdef ADXL345DEBUG
	printf_P(PSTR("adxl345 eeprom gravity:%hd bias:%hd,%hd,%hd\n"),
			 g_ee.gravity, g_ee.acc_bias[0], g_ee.acc_bias[1], g_ee.acc_bias[2]);
	puts_P(PSTR("adxl345 setup"));
#endif
}

// store the bias and gravity into eeprom
void adxl345_store_eeprom(void)
{
	eeprom_write_block(&g_ee, &s_ee, sizeof(struct adxl345_eeprom_t));
#ifdef ADXL345DEBUG
	printf("Storing adxl345 gravity:%hd bias:%hd,%hd,%hd\n", g_ee.gravity,
		   g_ee.acc_bias[0], g_ee.acc_bias[1], g_ee.acc_bias[2]);
#endif
}

// do a quick test of the device, see if it is ok
uint8_t adxl345_self_test(void)
{
	uint8_t buf;
	/* read back device id */
	if (adxl345_read(ADXL_DEVID, &buf, 1))
		return 1;
	// if correct, return 0
	return !(buf == 0xe5);
}

// find what the current bias of the device axis is right now
void adxl345_read_bias(void)
{
	int16_t bias[3] = {0,0,0};
	
	for (int i=0; i<16; ++i)
	{
		adxl345_read_accel();
		bias[0] += raw_acc[0];
		bias[1] += raw_acc[1];
		bias[2] += raw_acc[2];
		_delay_ms(20);
	}

	// average
	for (int i=0; i<3; ++i)
		bias[i] /= 16;

#ifdef ADXL345DEBUG
	printf_P(PSTR("ADXL345 bias:%hd,%hd,%hd\n"),
			 bias[0], bias[1], bias[2] - g_ee.gravity);
#endif
	
	// update eeprom values
	g_ee.acc_bias[0] = bias[0];
	g_ee.acc_bias[1] = bias[1];
	g_ee.acc_bias[2] = bias[2];
	acc_bias[0] = g_ee.acc_bias[0];
	acc_bias[1] = g_ee.acc_bias[1];
	// remove gravity from z axis
	acc_bias[2] = g_ee.acc_bias[2] - g_ee.gravity;
}

void adxl345_read_accel(void)
{
    uint8_t buf[6];

	/* read values */
	if (adxl345_read(ADXL_DATAX0, buf, 6))
		failed(4);

	// get axis offsets
	int x = s_device->axis_map[0];
	int y = s_device->axis_map[1];
	int z = s_device->axis_map[2];

	// get raw accelerations
    raw_acc[0] = (buf[1] << 8) | buf[0];
    raw_acc[1] = (buf[3] << 8) | buf[2];
    raw_acc[2] = (buf[5] << 8) | buf[4];

	// corrected accelerations, with sign and axis mapping
    corrected_acc[0] = s_device->sign_map[0] * raw_acc[x] - acc_bias[x];
    corrected_acc[1] = s_device->sign_map[1] * raw_acc[y] - acc_bias[y];
    corrected_acc[2] = s_device->sign_map[2] * raw_acc[z] - acc_bias[z];
}

static void adxl345_read_internal(int16_t* data)
{
	uint8_t buf[6];
	/* read values */
	if (adxl345_read(ADXL_DATAX0, buf, 6))
		failed(4);
	
    data[0] += (buf[1] << 8) | buf[0];
    data[1] += (buf[3] << 8) | buf[2];
    data[2] += (buf[5] << 8) | buf[4];
}

float adxl345_accel(int axis)
{
	return (float)corrected_acc[axis];
}

void adxl345_internal_self_test(void)
{
	int16_t results[6] = {0,0,0,0,0,0};
	uint8_t b;
	
	// set the device to full resolution, 16 g's
	b = 0x0b;
	if (adxl345_write(ADXL_DATA_FORMAT, &b, 1))
		failed(5);
	_delay_ms(20);

	// take and average 10 samples
	for (int j=0; j<10; ++j) {
		adxl345_read_internal(results);
		_delay_ms(10);
	}

	// now set the self-test bit
	b = 0x8b;
	if (adxl345_write(ADXL_DATA_FORMAT, &b, 1))
		failed(5);
	_delay_ms(20);

	// do 4 samples to let it settle
	for (int j=0; j<4; ++j) {
		adxl345_read_internal(&results[3]);
		_delay_ms(10);
	}
	
	// take and average 10 samples
	results[3] = results[4] = results[5] = 0;
	for (int j=0; j<10; ++j) {
		adxl345_read_internal(&results[3]);
		_delay_ms(10);
	}

	// clear the self-test bit
	b = 0x0a;
	if (adxl345_write(ADXL_DATA_FORMAT, &b, 1))
		failed(5);

	// average the values
	for (int i=0; i<6; ++i) {
		results[i] /= 10;
	}

#ifdef ADXL345DEBUG
	// print out the results
	printf_P(PSTR("Self-test adxl345 function:%hd,%hd,%hd\n"),
			 results[3] - results[0],
			 results[4] - results[1],
			 results[5] - results[2]);
#endif
}

