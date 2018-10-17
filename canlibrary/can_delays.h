

#ifndef __CAN_DELAYS_H
#define __CAN_DELAYS_H

#include <stdint.h>

void DelayMs( float delay );
void DelayUs( float delay );

#ifdef __ARM_ARCH

extern volatile uint32_t systick_millis_count;

static inline uint32_t millis(void) __attribute__((always_inline, unused));
static inline uint32_t millis(void)
{
	volatile uint32_t ret = systick_millis_count; // single aligned 32 bit is atomic;
	return ret;
}

#endif

#ifdef __AVR

#include <util/atomic.h>
#include "timer.h"

extern volatile uint32_t jiffies;

static inline uint32_t millis(void) __attribute__((always_inline, unused));
static inline uint32_t millis(void)
{
	uint32_t ret;
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
		ret = jiffies * 10;
	}
	return ret;
}

#endif

uint32_t millis(void);
uint32_t micros(void);

#endif
