/*------------------------------------------------*/
/* watchdog functions                             */

#include "defs.h"
#include <avr/io.h>
#include <avr/wdt.h>
#include <avr/eeprom.h>
#include <avr/pgmspace.h>
#include <stdio.h>
#include "watchdog.h"

/* ---------------------------------------------- */

// reset flags recorded at start
uint8_t g_reset_flags;

// initialized structure for eeprom to store reset counts
struct reset_count s_reset_count EEMEM = {
	0, 0, 0, 0
};

// global variable holding eeprom values for reset counts
struct reset_count g_reset_count;

/* start watchdog */

void watchdog_init(int watchdog_clock)
{
	// get the mcu reset status register and save it
	g_reset_flags = MCUSR;

	// reset it
	MCUSR = 0;

	// set watchdog to go off
	wdt_enable(watchdog_clock);
}

/* Initialize watchdog */

void watchdog_reset_count_update()
{
	// read the eeprom for reset counts
	eeprom_read_block(&g_reset_count, &s_reset_count,
					  sizeof(struct reset_count));
    // if eeprom hasn't been initialized, initialize it
    if (g_reset_count.watchdog_resets == -1) {
        g_reset_count.watchdog_resets = 0;
        g_reset_count.brown_out_resets = 0;
        g_reset_count.external_resets = 0;
        g_reset_count.power_on_resets = 0;
        eeprom_write_block(&g_reset_count, &s_reset_count, sizeof(struct reset_count));
    }
	// set the new reset counts
	if (g_reset_flags & _BV(WDRF))
		g_reset_count.watchdog_resets += 1;
	if (g_reset_flags & _BV(BORF))
		g_reset_count.brown_out_resets += 1;
	if (g_reset_flags & _BV(EXTRF))
		g_reset_count.external_resets += 1;
	if (g_reset_flags & _BV(PORF))
		g_reset_count.power_on_resets += 1;
	// write the eeprom for reset counts
	eeprom_write_block(&g_reset_count, &s_reset_count,
					   sizeof(struct reset_count));
}

/* print resets to uart */

void watchdog_print_flags()
{
	// print resets
    printf_P(PSTR("Reset Counts:\n"));
#if defined(__AVR_AT90CAN32__) || defined(__AVR_ATmega644__)
	if (g_reset_flags & _BV(JTRF))
		puts_P(PSTR("JTAG reset flag set"));
#endif
	if (g_reset_flags & _BV(WDRF))
		puts_P(PSTR("Watchdog reset flag set"));
	if (g_reset_flags & _BV(BORF))
		puts_P(PSTR("Brown-out reset flag set"));
	if (g_reset_flags & _BV(EXTRF))
		puts_P(PSTR("External reset flag set"));
	if (g_reset_flags & _BV(PORF))
		puts_P(PSTR("Power-on reset flag set"));
	printf_P(PSTR("Watchdog: %d Brown-out: %d\nExternal: %d Power-on: %d\n"),
			 g_reset_count.watchdog_resets, g_reset_count.brown_out_resets,
			 g_reset_count.external_resets, g_reset_count.power_on_resets);
	
}

/* reset the watchdog */

void watchdog_reset ()
{
	wdt_reset();
}
