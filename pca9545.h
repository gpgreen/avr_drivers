#ifndef PCA9545_H_
#define PCA9545_H_

#include <inttypes.h>

// define to 1 if debugging
/* #define PCA9545DEBUG   (1) */

// define if the reset pin is enabled
/* #define PCA9545_RESET (1) */

// return error codes
#define PCA9545_OK      (0)
#define PCA9545_FAIL    (1)
#define PCA9545_COMFAIL (2)

/* I2C address
 * There are 3 parts which have different addresses, plus 2 of the pins
 * are address pins. They can be grounded or pulled to VDD to select
 * different complete addresses.
 * To select a configuration combine a base with the A0,A1 pin selection
 * This is defined in PCA9545_I2C_ADDRESS in the application firmware.
 */

/* chip variations */
#define PCA9545A_BASE (0xe0)
#define PCA9545B_BASE (0xd0)
#define PCA9545C_BASE (0xb0)

/* AO,A1 grounded 11100xxb */
#define PCA9545_A0_A1_GND_ADDRESS   (0x00)
/* AO to VDD */
#define PCA9545_A0_VDD_A1_GND_ADDRESS   (0x2)
/* A1 to VDD */
#define PCA9545_A0_GND_A1_VDD_ADDRESS   (0x4)
/* A0,A1 to VDD */
#define PCA9545_A0_A1_VDD_ADDRESS   (0x6)

/* in defs.h, define the following to select the address configuration
 * #define PCA9545_I2C_ADDRESS (PCA9545A_BASE | A0_A1_GND_ADDRESS) */

////////////////////////////////////////////////////////////////////////////

/*
 * define bit patterns to select channel
 * they can be OR'd together, but don't overload bus and select
 * too many channels, per the datasheet
 */
#define PCA9545_CHAN1 0x1
#define PCA9545_CHAN2 0x2
#define PCA9545_CHAN3 0x4
#define PCA9545_CHAN4 0x8

/* function to call when an interrupt is received...
 * this function will be called in an interrupt context, so should be
 * short. It should set things up so that it will be called again to
 * remove the interrupt
 */
typedef void (*interrupt_fn_t)(void* userdata);

/* device structure */
struct pca9545_dev_t
{
	// which channel is current selected
	uint8_t channel_selected;
	// we have four channels, define an interrupt fn for each channel
	interrupt_fn_t interrupt_fn[4];
	// we can define user data to pass to the interrupt fn, can be NULL
	void* interrupt_data[4];
};

////////////////////////////////////////////////////////////////////////////
// methods

// the initialization fn
extern int pca9545_init(void);

// return PCA9545_OK, PCA9545_COMFAIL if spi communication failure,
// PCA9545_FAIL if bad channel argument
// the channel argument is a bitwise-OR of the channel defines
extern int pca9545_select_channel(struct pca9545_dev_t* device, uint8_t channel);

#if defined(PCA9545_RESET)

// reset the switch
extern void pca9545_reset(struct pca9545_dev_t* device);

#endif

// interrupt fn, call when interrupt received from this device
extern void pca9545_interrupt(struct pca9545_dev_t* device);

#endif  // PCA9545_H_
