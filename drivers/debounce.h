#ifndef DEBOUNCE_H_
#define DEBOUNCE_H_

#include <defs.h>
#include <stdint.h>

typedef struct debounce_pin 
{
    volatile uint8_t* ddr;          /* the DDR address for the debounced pin */
    volatile uint8_t* port;         /* the PORT address for the debounced pin */
    volatile uint8_t* pin;          /* the PIN address of the debounced pin */
    uint8_t pin_no;             /* the pin number of the debounced pin */
    uint8_t buffer;             /* the debounce buffer */
} debounce_pin_t;

// initialize the structure after port and pin have been set
extern void debounce_init(debounce_pin_t* db, int pull_up_on);

// call this in a timer interrupt or similar to debounce the pin
extern void debounce(debounce_pin_t* db);

// functions to see debounced state
extern int debounce_is_high(const debounce_pin_t* db);

extern int debounce_is_low(const debounce_pin_t* db);

#endif // DEBOUNCE_H_
