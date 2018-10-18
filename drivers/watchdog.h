#ifndef WATCHDOG_H_
#define WATCHDOG_H_

#include <inttypes.h>
#include <avr/wdt.h>

// reset source flags
extern uint8_t g_reset_flags;

// reset counts structure
struct reset_count
{
	uint16_t watchdog_resets;
	uint16_t brown_out_resets;
	uint16_t external_resets;
	uint16_t power_on_resets;
};

// reset counts
extern struct reset_count g_reset_count;

// watchdog functions
extern void watchdog_init(int watchdog_clock);
extern void watchdog_reset_count_update(void);
extern void watchdog_print_flags(void);
extern void watchdog_reset(void);

#endif
