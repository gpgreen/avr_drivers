#include <stdio.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
#include "scp1000.h"
#include "scp1000_def.h"
#include "spi.h"

/*-----------------------------------------------------------------------*/

// globals

// data structure for each device
scp1000_data_t g_scp1000_data[SCP_COUNT];

// save the operation code for setting device mode
static uint8_t s_opmode;

/*-----------------------------------------------------------------------*/

// select the device for SPI
inline static void set_cs_low(int device)
{
#ifdef SCP_CONFIG_DUAL
    if (device == 0)
#endif
        PORT_CB1 &= ~(_BV(P_CB1));
#ifdef SCP_CONFIG_DUAL
    else
        PORT_CB2 &= ~(_BV(P_CB2));
#endif
}

// select the device for SPI
inline static void set_cs_hi(int device)
{
#ifdef SCP_CONFIG_DUAL
    if (device == 0)
#endif
        PORT_CB1 |= _BV(P_CB1);
#ifdef SCP_CONFIG_DUAL
    else
        PORT_CB2 |= _BV(P_CB2);
#endif
}

/*-----------------------------------------------------------------------*/

/*
 * write a register on the SCP1000
 */
static void scp1000_write_register( int device, uint8_t address, uint8_t data )
{
#ifdef SCPDEBUG
	printf_P(PSTR("write_register:%d,%x,%x\n"), device, address, data);
#endif
    // set CS on SCP1000 low
    set_cs_low(device);

    spi_transfer((address << 2) | 0x2);
    spi_transfer(data);
   
    // set CS hi
    set_cs_hi(device);
#ifdef SCPDEBUG
	printf_P(PSTR("done write_register\n"));
#endif
}

/*-----------------------------------------------------------------------*/

/*
 * read a register on the SCP1000
 */
static uint8_t scp1000_read_register(int device, uint8_t address)
{
    uint8_t data;
   
#ifdef SCPDEBUG
	printf_P(PSTR("read_register:%d,%x\n"), device, address);
#endif

    // set CS on SCP1000 low
    set_cs_low(device);
   
    spi_transfer(address << 2);
    data = spi_transfer(0xff); 
   
    // CS hi
    set_cs_hi(device);
   
#ifdef SCPDEBUG
	printf_P(PSTR("done read_register:%x\n"), data);
#endif

    return data;
}

/*-----------------------------------------------------------------------*/

/*
 * read a 16bit register on the SCP1000
 */
static uint16_t scp1000_read_register16(int device, uint8_t address)
{
    uint16_t data;
   
#ifdef SCPDEBUG
	printf_P(PSTR("read_register16:%d,%x\n"), device, address);
#endif

    // set CS on SCP1000 low
    set_cs_low(device);
   
    spi_transfer(address << 2);
    data = spi_transfer(0xff) << 8; 
    data |= spi_transfer(0xff);
    
    // CS hi
    set_cs_hi(device);
   
#ifdef SCPDEBUG
	printf_P(PSTR("done read_register16:%x\n"), data);
#endif

    return data;
}

/*-----------------------------------------------------------------------*/

// given a user-selected operating mode, get the right op code
static uint8_t get_op_mode_code(SCP_OP_MODE omode)
{
	uint8_t code;
	switch (omode) {
	case HI_RES:
		code = SCP_OP_HI_RES_MODE;
#ifdef SCPDEBUG
		printf_P(PSTR("scp1000 hi-resolution mode.\n"));
#endif
		break;
	case HI_SPEED:
		code = SCP_OP_HI_SPD_MODE;
#ifdef SCPDEBUG
		printf_P(PSTR("scp1000 hi-speed mode.\n"));
#endif
		break;
	case ULTRA_LOW_POWER:
		code = SCP_OP_ULTRA_LO_PWR_MODE;
#ifdef SCPDEBUG
		printf_P(PSTR("scp1000 ultra-low-power mode.\n"));
#endif
		break;
	case LOW_POWER:
		code = SCP_OP_LOW_PWR_MODE;
#ifdef SCPDEBUG
		printf_P(PSTR("scp1000 low-power mode.\n"));
#endif
		break;
	default:
		code = SCP_OP_SELF_TEST;
#ifdef SCPDEBUG
		printf_P(PSTR("scp1000 self-test mode.\n"));
#endif
		break;
	}
	return code;
}

/*-----------------------------------------------------------------------*/

/*
 * Initialize the SCP1000
 */
uint8_t scp1000_init(SCP_OP_MODE omode)
{
#ifdef SCPDEBUG
	printf_P(PSTR("scp1000 init\n"));
#endif

	// set spi to low speed
	spi_lospeed();
	
	s_opmode = get_op_mode_code(omode);
	
	for (int i=0; i<SCP_COUNT; ++i) {
		
		// SCP1000 software reset
		// this puts device in standby mode
		scp1000_write_register(i, SCP_RSTR, 0x01);
    
		// now delay 60ms
		_delay_ms(60);

		// read STATUS every 10ms until LSB is 0, that means device is now
		// in standby mode
		uint8_t device = 0;
		int j = 0;
		do {
			uint8_t status = scp1000_read_register(i, SCP_STATUS);
			if (bit_is_clear(status, SCP_P_STARTUP))
				device = 1;
			_delay_ms(10);
		}
		while (++j != 6 && !device);
		if (j == 6) {
#ifdef SCPDEBUG
			printf_P(PSTR("Failed to set standby mode\n"));
#endif
			return SCP1000_FAIL;
		}

		// Now read EEPROM status
		uint8_t eeps = scp1000_read_register(i, SCP_DATARD8);
		if (eeps != 0x01) {
#ifdef SCPDEBUG
			printf_P(PSTR("bad EEPROM status:%x\n"), eeps);
#endif
			return SCP1000_FAIL;
		}
		
		// Configure the devices to high-resolution data acquistion
		scp1000_write_register( i, SCP_ADDPTR, SCP_CFG );
		scp1000_write_register( i, SCP_DATAWR, SCP_17BIT_RES );
		scp1000_write_register( i, SCP_OPERATION, s_opmode );

	} /* end device loop */
	
    // delay required by indirect write
    _delay_ms(50);

    // enable interrupts on atmega
    SCP_SW_DRDY0_MASK_REG |= _BV(SCP_SW_DRDY0_INT); /* pin change interrupt DRDY2 */
    SCP_SW_DRDY0_REG |= _BV(SCP_SW_DRDY0_ENABLE);	/* Pin change interrupt enable */
#ifdef SCP_CONFIG_DUAL
    SCP_SW_DRDY1_MASK_REG |= _BV(SCP_SW_DRDY1_INT);	/* pin change interrupt DRDY*/
	SCP_SW_DRDY1_REG |= _BV(SCP_SW_DRDY1_ENABLE);   /* Pin change interrupt enable */
#endif

#ifdef SCPDEBUG
	printf_P(PSTR("scp1000 init done\n"));
#endif

	return SCP1000_OK;
}

/*-----------------------------------------------------------------------*/

/*
 * Returns SCP1000_OK if ok,
 * SCP1000_FAIL if failed
 */
uint8_t scp1000_self_test(void)
{
#ifdef SCPDEBUG
	printf_P(PSTR("scp1000 self test.\n"));
#endif

	// set spi to lo speed
	spi_lospeed();
	
	for (int i=0; i<SCP_COUNT; ++i) {
		
		// run operation
		scp1000_write_register(i, SCP_OPERATION, SCP_OP_SELF_TEST);
		while (1)
		{
			uint8_t stat = scp1000_read_register(i, SCP_STATUS);
			if (bit_is_clear(stat, SCP_P_OPSTATUS))
				break;
		}
		// read result
		if (scp1000_read_register(i, SCP_DATARD8) != 0x01)
			return SCP1000_FAIL;

		// now set the operation mode again
		scp1000_write_register(i, SCP_OPERATION, s_opmode);
	}

#ifdef SCPDEBUG
	printf_P(PSTR("scp1000 self test ok.\n"));
#endif

	return SCP1000_OK;
}

/*-----------------------------------------------------------------------*/

/*
 * Read the temperature & pressure data from the selected device
 */
static void scp1000_read_data_i(int device)
{
#ifdef SCPDEBUG
	printf_P(PSTR("scp1000_read_data_i:%d\n"), device);
#endif

	// set spi to lo speed
	spi_lospeed();
	
    scp1000_data_t* buf = &g_scp1000_data[device];
    uint16_t tempbuf = scp1000_read_register16( device, SCP_TEMPOUT );
    uint8_t press_hibits = scp1000_read_register( device, SCP_DATARD8 );
    uint16_t press_lobits = scp1000_read_register16( device, SCP_DATARD16 );
	
    // convert temperature via formula in datasheet to degrees C
    // bit 13 is sign bit, if 1 then temp is negative 2's complement
    if ((tempbuf & 0x2000) == 0x2000) {
        // invert
        tempbuf ^= 0x3fff;
        // add 1
        buf->temp = -((tempbuf & 0x3fff) + 1);
    }
    else
        buf->temp = (tempbuf & 0x3fff);
	buf->temp /= 20.0;
	// convert temp to degrees Kelvin
	buf->temp += 273.15;
	// convert the pressure via formula in datasheet
	buf->press = press_lobits + press_hibits * 65536;
	buf->press /= 4.0;
}

/*-----------------------------------------------------------------------*/

/*
 * Read the temperature & pressure data from each device
 */
void scp1000_read_data(void)
{
    scp1000_read_data_i(0);
#ifdef SCP_CONFIG_DUAL
    scp1000_read_data_i(1);
#endif
}
