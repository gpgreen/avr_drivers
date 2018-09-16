#ifndef MCP2515_H_
#define MCP2515_H_

#include "defs.h"
#include <inttypes.h>
#include "can.h"

// define to 1 if debugging
/* #define MCPDEBUG   (1) */

// we need to define how big the FIFO's buffers are
// we have a receive FIFO and an error FIFO
// the sizes are board dependent because of available RAM

#ifndef MCP_RECVBUFLEN
#error "This driver requires a MCP_RECVBUFLEN define for length of receive FIFO"
#endif

#ifndef MCP_ERRBUFLEN
#error "This driver requires a MCP_ERRBUFLEN define for length of error FIFO"
#endif

// return error codes
#define MCP2515_OK      (0)
#define MCP2515_FAIL    (1)
#define MCP_ALLTXBUSY   (2)

// we need to define the MCP clock
// this is not the same as the CPU
// define one of these in the board specific firmware
/* #define MCP2515_8MHZ    (1) */
/* #define MCP2515_16MHZ   (1) */

// check
#ifndef MCP2515_8MHZ
#ifndef MCP2515_16MHZ
#error "This driver requires a MCP2515_XXXMHZ define. Choose one."
#endif
#endif

/*-----------------------------------------------------------------------*/

// define an interrupt vector for software interrupt
#ifndef MCP2515_INT_VECT
#error "This driver requires a MCP2515_INT_VECT define to specify interrupt vector"
#endif

/* define if this is a general interrupt, where it happens any time the pin
 * changes state. This means we have to check the PINn value for a low level
 * on the interrupt, as opposed to it only happening when the level goes low
 */
/* #define MCP2515_INT_VECT_ANY_CHANGE 1 */

/* struct for mcp2515 device specific data */
struct mcp2515_dev_priv {
	// set to 1 if interrupt happened, 0 if not
	volatile uint8_t flag;
	// capture pointer to settings
	can_init_t* settings;
	// interrupt control register, controlling edge detection
	volatile uint8_t * int_dir_reg;
	// mask for interrupt control register for edge detection
	uint8_t int_dir_mask;
	// interrupt control register, enable/disable
	volatile uint8_t * int_en_reg;
	// mask for interrupt control register
	uint8_t int_en_mask;
	// GPIO port register for interrupt
	volatile uint8_t * port;
	// GPIO port direction control register for interrupt
	volatile uint8_t * ddr_port;
#if defined(MCP2515_INT_VECT_ANY_CHANGE)
	// GPIO port pin register for interrupt
	volatile uint8_t * pin;
#endif
	// GPIO pin mask for interrupt
	uint8_t port_pin;
};

// device specific struct
extern struct mcp2515_dev_priv g_mcp2515_dev_priv;

// the CAN device
extern struct can_device mcp2515_dev;

// initialize the device
extern int mcp2515_init(can_init_t* settings);

// run self test on device
extern int mcp2515_self_test(void);

// have we received a msg
extern int mcp2515_check_receive(void);

// read receive buf status (SPI command)
//extern uint8_t mcp2515_read_rx_status(void);

// find a free tx buf
extern int mcp2515_get_next_free_tx_buf(int* txbuf_n);

// send a can message to the device
extern void mcp2515_write_msg(int buffer_sidh_addr, const can_msg_t* msg);

// tell the device to send the message in the tx buf
//extern void mcp2515_start_transmit(const uint8_t txbuf_n);

// read a can message received by device
extern int mcp2515_read_msg(can_msg_t* msg);

// called when device interrupts
extern int mcp2515_handle_interrupt(int* status_flag);

// called to get rx,tx error counts
extern int mcp2515_error_counts(uint8_t* tx_count, uint8_t* rx_count);

// clear the tx buffers of any unsent messages
extern void mcp2515_clear_tx_buffers(void);

#endif
