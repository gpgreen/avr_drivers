#ifndef AT90CAN_H_
#define AT90CAN_H_

#include "defs.h"
#include <inttypes.h>
#include "can.h"

// define to 1 if debugging
/* #define AT90CANDEBUG   (1) */

// we need to define how big the FIFO's buffers are
// we have a receive FIFO and an error FIFO
// the sizes are board dependent because of available RAM

#ifndef AT90CAN_RECVBUFLEN
#error "This driver requires a AT90CAN_RECVBUFLEN define for length of receive FIFO"
#endif

#ifndef AT90CAN_ERRBUFLEN
#error "This driver requires a AT90CAN_ERRBUFLEN define for length of error FIFO"
#endif

// return error codes
#define AT90CAN_OK          (0)
#define AT90CAN_FAIL        (1)
#define AT90CAN_ALLTXBUSY   (2)
#define AT90CAN_NOMSG       (15)

// we need to define the AT90CAN clock
// this is not the same as the CPU
// define one of these in the board specific firmware
/* #define AT90CAN_8MHZ    (1) */
/* #define AT90CAN_16MHZ   (1) */

// check
#ifndef AT90CAN_8MHZ
#ifndef AT90CAN_16MHZ
#error "This driver requires a AT90CAN_XXXMHZ define. Choose one."
#endif
#endif

// the CAN device
extern struct can_device at90can_dev;

// initialize the device
extern uint8_t at90can_init(can_init_t* settings);

// run self test on device
extern uint8_t at90can_self_test(void);

// find a free tx buf
extern uint8_t at90can_get_next_free_tx_mob(int *txbuf_n);

// send a can message to the device
extern void at90can_write_msg(int tx_mob_idx, const can_msg_t* msg);

// check for mob that has a received can message
extern uint8_t at90can_check_msg_received(void);

// read a can message received by device
extern uint8_t at90can_read_msg(can_msg_t* msg);

// called to see if device interrupted, also does interrupt work
extern uint8_t at90can_handle_interrupt(uint8_t* status_flag);

// called to get rx,tx error counts
extern uint8_t at90can_error_counts(uint8_t* tx_count, uint8_t* rx_count);

// clear the tx buffers of any unsent messages
extern void at90can_clear_tx_buffers(void);

#endif
