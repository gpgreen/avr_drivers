/**
 * swap values to/from host little-endian to network big-endian
 */
#include "conversion.h"

/*-----------------------------------------------------------------------*/

void convert_float_to_big_endian(float val, uint8_t *dest)
{
	uint8_t *fval = (uint8_t*)&val;
	dest[0] = fval[3];
	dest[1] = fval[2];
	dest[2] = fval[1];
	dest[3] = fval[0];
}

void convert_ushort_to_big_endian(uint16_t val, uint8_t *dest)
{
	uint8_t *sval = (uint8_t*)&val;
	dest[0] = sval[1];
	dest[1] = sval[0];
}

void convert_long_to_big_endian(int32_t val, uint8_t *dest)
{
	uint8_t *ival = (uint8_t*)&val;
	dest[0] = ival[3];
	dest[1] = ival[2];
	dest[2] = ival[1];
	dest[3] = ival[0];
}

void read_long_as_big_endian(int32_t *val, uint8_t *buf)
{
	uint8_t *tmp = (uint8_t*)val;

	tmp[0] = buf[3];
	tmp[1] = buf[2];
	tmp[2] = buf[1];
	tmp[3] = buf[0];
}

void read_float_as_big_endian(float *val, uint8_t *buf)
{
	uint8_t *tmp = (uint8_t*)val;

	tmp[0] = buf[3];
	tmp[1] = buf[2];
	tmp[2] = buf[1];
	tmp[3] = buf[0];
}

