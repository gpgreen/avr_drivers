#ifndef CAN_H_
#define CAN_H_

#include "defs.h"
#include <inttypes.h>

// uncomment for debugging
//#define CANDEBUG       1

// uncomment  for timestamps in message structs
//#define CAN_TSTAMP     1

// speed settings
#define CAN_20KBPS     (0)
#define CAN_100KBPS    (1)
#define CAN_125KBPS    (2)
#define CAN_250KBPS    (3)
#define CAN_500KBPS    (4)
#define CAN_1000KBPS   (5)

// can function error codes
#define CAN_OK         (0)
#define CAN_FAILINIT   (1)
#define CAN_FAILTX     (2)
#define CAN_MSGAVAIL   (3)
#define CAN_NOMSG      (4)
#define CAN_CTRLERROR  (5)
#define CAN_ALLTXBUSY  (6)
#define CAN_INTERRUPT  (7)
#define CAN_NOINTERRUPT (8)
#define CAN_FAIL       (255)

// can hardware error codes
#define CAN_BUS_PASSIVE (1)
#define CAN_BUS_OFF (2)
#define CAN_MSG_BIT_ERR (3)
#define CAN_MSG_STUFF_ERR (4)
#define CAN_CRC_ERR (5)
#define CAN_FRAME_ERR (6)
#define CAN_ACK_ERR (7)
#define CAN_RECV_BUF_OVERFLOW (8)
#define CAN_ERROR_WARNING (9)

// maximum data bytes for can device commands
#define CAN_DEVICE_CMD_MAX_DATA    8

// can device commands
#define CAN_LOOPBACK_ENABLE        0
#define CAN_LOOPBACK_ENABLE_SZ     1

#define CAN_BAUDRATE               1
#define CAN_BAUDRATE_SZ            1

#define CAN_TX_TIMEOUT_MS          2
#define CAN_TX_TIMEOUT_MS_SZ       2

enum can_bus_state {
    OK, ERROR_ACTIVE, ERROR_PASSIVE, BUS_OFF 
};

// type defines for message id types
#define CAN_STANDARD_ID                 0
#define CAN_EXTENDED_ID                 1

#define CAN_STANDARD_ID_MASK            0x7ff
#define CAN_EXTENDED_ID_MASK            0x1fffffff

// the message itself
typedef struct
{
  // extended lower 29 bits, or standard lower 11 bits
  uint32_t  id;
  uint8_t   idtype;           // CAN_STANDARD_ID or CAN_EXTENDED_ID
  uint8_t   rtr;
  uint8_t   length;
  uint8_t   data[8];
#ifdef CAN_TSTAMP
  uint32_t  tstamp;
#endif
} can_msg_t;

// device command
struct can_device_command
{
  uint8_t command;
  uint8_t length;
  uint8_t data[CAN_DEVICE_CMD_MAX_DATA];
};

// filter structure
typedef struct
{
  uint8_t filtering_on;
  uint8_t num_filters;
  uint16_t standard_id_mask[15];
  uint16_t extended_id_mask[15];
  uint16_t standard_id_filter[15];
  uint16_t extended_id_filter[15];
} can_filter_t;

// error structure
typedef struct
{
  uint8_t error_code;
  uint8_t dev_buffer;
  uint8_t dev_code;
} can_error_t;

// initialization structure
typedef struct
{
  uint8_t speed_setting;
  uint8_t loopback_on;
  uint16_t tx_wait_ms;
  can_filter_t filters;
} can_init_t;

// hardware specific functions
struct can_device
{
  void *priv_dev;
  can_init_t *settings;
  int devno;

  // state of the CAN bus
  enum can_bus_state bus_state;

  // initialization
  // returns CAN_OK, or CAN_FAILINIT
  int (*init_fn)(can_init_t* settings, struct can_device *dev);

  // re-initialization, used when changing settings after device
  // has been setup once already
  // returns CAN_OK, or CAN_FAILINIT
  int (*reinit_fn)(struct can_device *dev);

  // returns CAN_OK, or CAN_FAIL
  int (*self_test_fn)(struct can_device *dev);

  // returns CAN_MSGAVAIL or CAN_NOMSG
  int (*check_receive_fn)(struct can_device *dev);

  // returns CAN_OK, or CAN_ALLTXBUSY
  int (*free_send_buffer_fn)(struct can_device *dev, int* buf_no);

  // send a can message using the given buffer
  void (*write_msg_fn)(struct can_device *dev,
		       int buf_no, const can_msg_t* msg);

  // returns CAN_OK if read message, CAN_NOMSG if none there
  int (*read_msg_fn)(struct can_device *dev,
		     can_msg_t* msg);

  // device command, returns CAN_OK, CAN_FAIL, CAN_FAILINIT
  int (*device_command)(struct can_device *dev,
			const struct can_device_command* dev_cmd);

  // returns CAN_INTERRUPT if interrupt has occurred,
  // CAN_NOINTERRUPT if no interrupt
  int (*handle_int_fn)(struct can_device *dev,
		       int *status_flag);

  // called when an error happens in CAN hardware
  void (*handle_error_fn)(struct can_device *dev, 
			  const can_error_t* err);

  // called to get error counts, returns CAN_OK, CAN_FAIL
  int (*error_counts)(struct can_device *dev, uint8_t* tx_count, 
		      uint8_t* rx_count);

  // clear the tx buffers of messages that haven't been sent
  void (*clear_tx_buffers)(struct can_device *dev);
};

// driver functions

extern int can_init(can_init_t* settings, struct can_device* dev);

extern int can_reinit(struct can_device* dev);

extern int can_self_test(struct can_device *dev);

extern void can_init_message_struct(can_msg_t* msg);

extern int can_check_receive(struct can_device *dev);

extern int can_send_message(struct can_device *dev, 
                            const can_msg_t* msg);

extern int can_read_message(struct can_device *dev, can_msg_t* msg);

extern int can_device_command(struct can_device *dev, 
                              const struct can_device_command* dev_cmd);

extern int can_handle_interrupt(struct can_device *dev, 
                                int *status_flag);

extern void can_handle_error(struct can_device *dev, 
                             const can_error_t* err);

extern int can_error_counts(struct can_device *dev, 
                            uint8_t *tx_count, uint8_t *rx_count);

extern void can_clear_tx_buffers(struct can_device *dev);

#endif
