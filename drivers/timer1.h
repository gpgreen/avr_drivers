#ifndef TIMER1_H_
#define TIMER1_H_

#include <stdint.h>

typedef void (*timer1_compare_cb)(void);

// Set the scaling in Timer1
enum timer1_scaling{
    NO_CLOCK,
    NO_SCALING,
    CLK8,
    CLK64,
    CLK256,
    CLK1024,
    CLKEXTFALLING,
    CLKEXTRISING
};

// Initialization structure for Timer1
// Only 1 of the compare callbacks can
// be used at a time, so compareA_val or
// compareB_val should be zero, ie not used
// the corresponding callback needs to be
// non-NULL for the timer to work
typedef struct
{
    enum timer1_scaling scale;
    timer1_compare_cb compareA_cb;
    uint16_t compareA_val;
    timer1_compare_cb compareB_cb;
    uint16_t compareB_val;
#if defined(__AVR_AT90CAN32__)
    timer1_compare_cb compareC_cb;
    uint16_t compareC_val;
#endif
} timer1_init_t;

// initialize the timer hardware
void timer1_init(timer1_init_t* settings);

#endif
