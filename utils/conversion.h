#ifndef CONVERSION_H_
#define CONVERSION_H_

#include "defs.h"
#include <inttypes.h>

/*-----------------------------------------------------------------------*/

// function to convert a float to big endian
extern void convert_float_to_big_endian(float val, uint8_t *dest);

// function to convert a short to big endian
extern void convert_ushort_to_big_endian(uint16_t val, uint8_t *dest);

// function to convert a long to big endian
extern void convert_long_to_big_endian(int32_t val, uint8_t *dest);

// function to read a long from big endian buffer
extern void read_long_as_big_endian(int32_t *val, uint8_t *buf);

// function to read a float from big endian buffer
extern void read_float_as_big_endian(float *val, uint8_t *buf);

/*-----------------------------------------------------------------------*/

#endif
