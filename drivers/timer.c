#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/atomic.h>
#include "timer.h"

/*-----------------------------------------------------------------------*/

volatile uint32_t jiffies;

/*
 * Timer compare output 0A interrupt
 */
#if defined(__AVR_AT90CAN32__)
ISR(TIMER0_COMP_vect)
#else
ISR(TIMER0_COMPA_vect)
#endif
{
	++jiffies;
}

// timer_init
//
// We want this timer to measure in units of tenth milliseconds
// or 100 usecs per unit
void timer_init(void)
{
    // CTC mode, prescale fosc / 64
#if defined(__AVR_AT90CAN32__) || defined(__AVR_ATmega328__)
	TCCR0A = _BV(WGM01) | _BV(CS01);
#else
    TCCR0A = _BV(WGM01);
    TCCR0B = _BV(CS01)|_BV(CS00);
#endif
    // we have the 8 bit timer set so that each compare match = .1ms
    OCR0A = F_CPU / 64 / 1000;
    // set OC interrupt 0A
    TIMSK0 = _BV(OCIE0A);
}

// jiffie
//
// return the current time in tenth milliseconds
uint32_t jiffie(void)
{
	uint32_t res;
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
	{
		res = jiffies;
	}
	return res;
}

// duration
//
// return the amount of time from start time in ms
uint32_t duration(uint32_t t0)
{
	uint32_t res;
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
	{
		res = jiffies;
	}
	// handle the case where the timer has overflowed
	if (t0 > res)
	{
		res = t0 - res;
	}
	else
		res -= t0;
	return res;
}

// timer_elapsed
//
// return the amount of time from t0 to t1 in ms
uint32_t timer_elapsed(uint32_t t0, uint32_t t1)
{
	uint32_t res;
	// handle the case where the timer has overflowed
	if (t0 > t1)
	{
		res = t0 - t1;
	}
	else
		res = t1 - t0;
	return res;
}


