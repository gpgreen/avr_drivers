#ifndef TIMER_H_
#define TIMER_H_

#include "defs.h"
#include <inttypes.h>

// initialize the timer hardware
void timer_init(void);

// get the current time in milliseconds
uint32_t jiffie(void);

// what is the elapsed time from t0 in milliseconds
uint32_t duration(uint32_t t0);

// what is the elapsed time from t0 to t1 in milliseconds
uint32_t timer_elapsed(uint32_t t0, uint32_t t1);

#endif
