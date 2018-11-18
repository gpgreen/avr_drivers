#include "defs.h"

#include <avr/io.h>
#include <avr/interrupt.h>
#include "timer1.h"

/*-----------------------------------------------------------------------*/

static timer1_init_t* s_settings;

/*
 * Timer compare output 1A interrupt
 */
ISR(TIMER1_COMPA_vect)
{
    s_settings->compareA_cb();
}

/*
 * Timer compare output 1B interrupt
 */
ISR(TIMER1_COMPB_vect)
{
    s_settings->compareB_cb();
}

/*
 * Timer compare output 1C interrupt
 */
#if defined(__AVR_AT90CAN32__)
ISR(TIMER1_COMPC_vect)
{
    s_settings->compareC_cb();
}
#endif

void timer1_init(timer1_init_t* settings)
{
    s_settings = settings;
    
    uint8_t divset = 0x0;
    switch (settings->scale)
    {
    case NO_CLOCK:
        return;
    case NO_SCALING:
        break;
    case CLK8:
        divset |= _BV(CS11);
        break;
    case CLK64:
        divset |= _BV(CS11)|_BV(CS10);
        break;
    case CLK256:
        divset |= _BV(CS12);
        break;
    case CLK1024:
        divset |= _BV(CS12)|_BV(CS10);
        break;
    case CLKEXTFALLING:
        divset |= _BV(CS12)|_BV(CS11);
        break;
    case CLKEXTRISING:
        divset |= _BV(CS12)|_BV(CS11)|_BV(CS10);
        break;
    }

    // CTC mode
    TCCR1B = _BV(WGM12) | divset;

    // set compare registers and init interrupts
    
    if (settings->compareA_val > 0
        && settings->compareA_cb != 0)
    {
        OCR1AH = settings->compareA_val >> 8;
        OCR1AL = settings->compareA_val & 0xff;
    
        // set OC interrupt 1A
        TIMSK1 |= _BV(OCIE1A);
    }
    
    else if (settings->compareB_val > 0
             && settings->compareB_cb != 0)
    {
        OCR1BH = settings->compareB_val >> 8;
        OCR1BL = settings->compareB_val & 0xff;
    
        // set OC interrupt 1B
        TIMSK1 |= _BV(OCIE1B);
    }
    
#if defined(__AVR_AT90CAN32__)
    else if (settings->compareC_val > 0
             && settings->compareC_cb != 0)
    {
        OCR1CH = settings->compareC_val >> 8;
        OCR1CL = settings->compareC_val & 0xff;
    
        // set OC interrupt 1C
        TIMSK1 |= _BV(OCIE1C);
    }
#endif
}
