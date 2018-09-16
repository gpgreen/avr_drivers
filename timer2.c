#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/atomic.h>
#include "timer2.h"

#include <stdio.h>
#include <avr/pgmspace.h>

/*-----------------------------------------------------------------------*/

static struct timer* s_timer_head;

/*-----------------------------------------------------------------------*/

/*
 * Timer 1 Compare A interrupt
 */
ISR(TIMER1_COMPA_vect)
{
	struct timer* t = s_timer_head;
	struct timer* tp = 0;

	while (t != 0) {
		if (t->state == Active && ++t->count == t->length) {
			if (tp == 0)
				s_timer_head = t->next;
			else
				tp->next = t->next;
			t->state = Done;
		}
		tp = t;
		t = t->next;
	}
}

/*-----------------------------------------------------------------------*/

static void timer_list_insert(struct timer* timer)
{
	puts_P(PSTR("timer_list_insert"));
	
	// periodic timer gets reinserted
	timer->state = Active;
	timer->count = 0;
	timer->next = 0;
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE) 
	{
		// case no timers yet
		if (s_timer_head == 0)
			s_timer_head = timer;
			
		// find the timer that is longer than new timer
		else {
			
			struct timer* current = s_timer_head;
			struct timer* prev = 0;
			while (current != 0) {
				// found it, fix the list
				if (timer->length < (current->length - current->count)) {
					if (prev == 0)
						s_timer_head = timer;
					else
						prev->next = timer;
					timer->next = current;
					break;
				}
				prev = current;
				current = current->next;
			}
			
			// no longer timer found, goes on the tail
			if (current == 0)
				prev->next = timer;
		}
	}
}

/*-----------------------------------------------------------------------*/

static void timer_list_remove(struct timer* timer)
{
	puts_P(PSTR("timer_list_remove"));

	ATOMIC_BLOCK(ATOMIC_RESTORESTATE) 
	{
		
		if (timer == s_timer_head)
			s_timer_head = s_timer_head->next;
		else {
			struct timer* current = s_timer_head;
			while (current->next != 0) {
				if (current->next == timer) {
					current->next = timer->next;
					break;
				}
				current = current->next;
			}
		}
	}
	/*printf_P(PSTR("timer_list_remove:%d\n"), code);*/
}
	
/*-----------------------------------------------------------------------*/

// timer2_init
//
// We want this timer to measure in units of microseconds
// every output compare match will be 1 ms
void timer2_init(void)
{
	// CTC mode, clk at F_CPU/8
	TCCR1B = _BV(WGM12) | _BV(CS11);
	// set output compare
	OCR1A = (F_CPU / 8 / 1000);
    // set output compare 1A
    TIMSK1 = _BV(OCIE1A);
}

/*-----------------------------------------------------------------------*/

// timer2_start
int timer2_start(struct timer* timer, uint16_t ms, enum TimerType ttype)
{
	if (timer->state != Idle)
		return -1;
	
	timer->state = Idle;
	timer->type = ttype;
	timer->length = ms;
	timer->count = 0;
	timer->next = 0;

	timer_list_insert(timer);

	return 0;
}

/*-----------------------------------------------------------------------*/

// timer2_finished
//
// returns -1 if done, 0 still running
int timer2_finished(struct timer* timer)
{
	return timer->state == Done? -1 : 0;
}

/*-----------------------------------------------------------------------*/

// timer2_wait
//
// returns -1 if done, 0 if error
int timer2_wait(struct timer* timer)
{
	puts_P(PSTR("timer2_wait"));
	// if not running, return error
	if (timer->state == Idle) {
		puts_P(PSTR("timer2_wait idle"));
		return 0;
	}
	
	puts_P(PSTR("timer2_wait1"));
	
	// wait for expiration
	while (timer->state != Done)
	{
		ATOMIC_BLOCK(ATOMIC_RESTORESTATE) 
		{
			struct timer* current = s_timer_head;
			while (current != 0) {
				printf_P(PSTR("state:%d type:%d l:%hd c:%hd\n"),
						 timer->state, timer->type, timer->length,
						 timer->count);
				current = current->next;
			}
		}
	}

	puts_P(PSTR("timer2_wait2"));

	// restart or idle the timer, depending on type
	if (timer->type == SingleShot)
		timer->state = Idle;
	else
		timer_list_insert(timer);

	return -1;
}

// timer2_cancel
//
// remove the timer from the list, also idle it
int timer2_cancel(struct timer* timer)
{
	timer->state = Idle;
	timer_list_remove(timer);
	return 0;
}

