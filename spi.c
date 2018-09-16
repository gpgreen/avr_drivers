#include "spi.h"

/*
 * Initialize the SPI interface
 *
 */
int spi_init(uint8_t clk_scale)
{
    // set pins for SPI interface
	// SCK MOSI are output, rest are input
    DDR_SPI |= _BV(P_SCK) | _BV(P_MOSI);
	// set levels on the port
    PORT_SPI &= ~(_BV(P_SCK) | _BV(P_MOSI));

	uint8_t clock_reg_val = _BV(SPE)|_BV(MSTR);
	switch (clk_scale) {
	case 2:
		SPSR = _BV(SPI2X);
		break;
	case 4:
		SPSR = 0;
		break;
	case 8:
		clock_reg_val |= _BV(SPR0);
		SPSR = _BV(SPI2X);
		break;
	case 16:
		clock_reg_val |= _BV(SPR0);
		SPSR = 0;
		break;
	case 32:
		clock_reg_val |= _BV(SPR1);
		SPSR = _BV(SPI2X);
		break;
	case 64:
		clock_reg_val |= _BV(SPR1);
		SPSR = 0;
		break;
	case 128:
		clock_reg_val |= _BV(SPR1)|_BV(SPR0);
		SPSR = 0;
		break;
	default:
		return SPI_FAILED;
	}
    // set SPI master, SPI enabled, and clock setting selected above
    SPCR = clock_reg_val;
    return SPI_OK;
}

/*
 * put/receive a byte on the SPI bus
 */
uint8_t spi_transfer( uint8_t data )
{
    // send this byte
    SPDR = data;
   
    // wait until byte is sent
    loop_until_bit_is_set(SPSR, SPIF);

    return SPDR;
}

