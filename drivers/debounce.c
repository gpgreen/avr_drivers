#include <avr/sfr_defs.h>
#include "debounce.h"

void debounce_init(debounce_pin_t* db, int pull_up_on)
{
    *db->ddr &= ~_BV(db->pin_no);
    if (pull_up_on) {
        *db->port |= _BV(db->pin_no);
        db->buffer = 0xFF;
    }
    else
        db->buffer = 0;
}

void debounce(debounce_pin_t* db)
{
    db->buffer <<= 1;
    db->buffer |= ((*db->pin & _BV(db->pin_no)) >> db->pin_no);
}

int debounce_is_high(const debounce_pin_t* db)
{
    return db->buffer == 0xFF;
}

int debounce_is_low(const debounce_pin_t* db)
{
    return db->buffer == 0;
}

