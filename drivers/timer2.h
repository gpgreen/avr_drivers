#ifndef TIMER2_H_
#define TIMER2_H_

#include "defs.h"
#include <inttypes.h>

// state of the timer, used internally
enum TimerState {Idle, Active, Done};

// which type of timer
enum TimerType {SingleShot, Periodic};

// structure for timer data
struct timer 
{
	volatile enum TimerState state;
	enum TimerType type;
	uint16_t length;
	volatile uint16_t count;
	struct timer* next;
};

// initialize the hardware
extern void timer2_init(void);
// start the a timer for a given length and type
extern int timer2_start(struct timer* timer, uint16_t ms, 
						enum TimerType ttype);
// is the timer finished, doesn't remove from list
extern int timer2_finished(struct timer* timer);
// wait for timer to finish, removes from list
extern int timer2_wait(struct timer* timer);
// cancel timer and remove from list
extern int timer2_cancel(struct timer* timer);

#endif
