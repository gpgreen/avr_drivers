#ifndef SCP1000_H_
#define SCP1000_H_

#include "defs.h"
#include <avr/io.h>

/*-----------------------------------------------------------------------*/

/* define if we want uart debugging output */
//#define SCPDEBUG 1

// return error codes
#define SCP1000_OK      (0)
#define SCP1000_FAIL    (1)
#define SCP_REALTMFAIL  (2)

/*-----------------------------------------------------------------------*/

// enumeration to choose which operating mode to use
typedef enum {
	HI_RES, 
	HI_SPEED, 
	ULTRA_LOW_POWER, 
	LOW_POWER
} SCP_OP_MODE;

/*-----------------------------------------------------------------------*/

// data structure to hold readings from device
typedef struct 
{
    // the temp in degrees K
    float temp;
    // the pressure in kPa
    float press;
} scp1000_data_t;

/*-----------------------------------------------------------------------*/

// if this is defined, we have 2 scp1000 chips on the board
#ifdef SCP_CONFIG_DUAL
#define SCP_COUNT 2
#else
#define SCP_COUNT 1
#endif

/*-----------------------------------------------------------------------*/

// data structures for device(s)
extern scp1000_data_t g_scp1000_data[SCP_COUNT];

/*-----------------------------------------------------------------------*/

extern uint8_t scp1000_init(SCP_OP_MODE omode);

extern uint8_t scp1000_self_test(void);

extern void scp1000_read_data(void);

/*-----------------------------------------------------------------------*/

#endif
