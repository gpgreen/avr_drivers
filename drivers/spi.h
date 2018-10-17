#ifndef SPI_H_
#define SPI_H_

#include "defs.h"
#include <inttypes.h>
#include <avr/io.h>

#define SPI_OK 0
#define SPI_FAILED 1

// initialize SPI - clk_scale [2-128]
// returns SPI_OK or SPI_FAILED
extern int spi_init(uint8_t clk_scale);

// send/receive a byte on SPI
extern uint8_t spi_transfer(uint8_t data);

#endif
