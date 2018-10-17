#include "defs.h"
#include <stdio.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
#include "pca9545.h"
#include "pca9545_def.h"
#include "globals.h"
#include <i2cmaster.h>

// globals
#ifdef PCA9545DEBUG
static const char* k_pcastr = "pca9545";
#endif

//#define PCA9545_I2C_ADDRESS 0xd4

////////////////////////////////////////////////////////////////////////////

/*
 * write control register on the PCA9545
 */
static int pca9545_write_register(uint8_t data )
{
#ifdef PCA9545CDEBUG
	printf_P(PSTR("write_register(%x):%x\n"), PCA9545_I2C_ADDRESS+I2C_WRITE,
			 data);
#endif

	i2c_start_wait(PCA9545_I2C_ADDRESS+I2C_WRITE);

	if (i2c_write(data))
		return -1;
	
	i2c_stop();
	
#ifdef PCA9545CDEBUG
	puts_P(PSTR("done write_register"));
#endif

	return 0;
}

/*
 * read control register on the PCA9545
 */
static int pca9545_read_register(uint8_t* data)
{
#ifdef PCA9545CDEBUG
	printf_P(PSTR("read_register(%x)\n"), PCA9545_I2C_ADDRESS+I2C_READ);
#endif

	i2c_start_wait(PCA9545_I2C_ADDRESS+I2C_READ);

	*data = i2c_readNak();
	
	i2c_stop();
	
#ifdef PCA9545CDEBUG
	printf_P(PSTR("done read_register:%x\n"), *data);
#endif

	return 0;
}

int pca9545_init(void)
{
	/* nominal 5ms power-up time */
	_delay_ms(5);

#ifdef PCA9545DEBUG
	printf_P(PSTR("%s:i2c address:%x\n"), k_pcastr, PCA9545_I2C_ADDRESS);
	printf_P(PSTR("%s:initializing\n"), k_pcastr);
#endif
	/* write the control register to reset state */
	if (pca9545_write_register(0x00))
        return PCA9545_FAIL;

#ifdef PCA9545DEBUG
	printf_P(PSTR("%s:done\n"), k_pcastr);
#endif
	return PCA9545_OK;
}

int pca9545_select_channel(struct pca9545_dev_t* device, uint8_t channel)
{

#ifdef PCA9545DEBUG
	printf_P(PSTR("%s:selch\n"), k_pcastr);
#endif

	// test for illegal bits set in channel value
	if((channel & 0xf0) > 0)
		return PCA9545_FAIL;

	if (pca9545_write_register(channel))
		return PCA9545_COMFAIL;

    // per datasheet, put a stop on the bus to enable channel.
    // this happens at the end of pca9545_write_register
    
	device->channel_selected = channel;
	
#ifdef PCA9545DEBUG
	printf_P(PSTR("%s:selch:%x\n"), k_pcastr, channel);
#endif
	return PCA9545_OK;
}

#if defined(PCA9545_RESET)

void pca9545_reset(struct pca9545_dev_t* device)
{
#ifdef PCA9545DEBUG
	printf_P(PSTR("%s:reset\n"), k_pcastr);
#endif
	
	PORT_I2CRESET &= ~(_BV(P_I2CRESET));
	_delay_ms(1);
	PORT_I2CRESET |= _BV(P_I2CRESET);
	device->channel_selected = 0;

#ifdef PCA9545DEBUG
	printf_P(PSTR("%s:reset finished\n"), k_pcastr);
#endif
}

#endif

void pca9545_interrupt(struct pca9545_dev_t* device)
{
	uint8_t buf;
	
	// read the interrupt flags..
	pca9545_read_register(&buf);
	for(int i=0; i<4; ++i) {
		int test_int = i+4;
		if((buf && _BV(test_int)) == _BV(test_int)
		   && device->interrupt_fn[i] != NULL)
			device->interrupt_fn[i](device->interrupt_data[i]);
	}
}


