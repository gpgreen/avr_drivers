#ifndef CANSERIAL_H_
#define CANSERIAL_H_

#include "defs.h"
#include <inttypes.h>
#include "can.h"

/*
 * format of commands and messages
 *
 * We will use same format as slcan linux serial can driver for sending/receiving
 * can frames:
 *
 * <type> <id> <dlc> <data>*
 *
 * Extended frames (29 bit) are defined by capital characters in the type.
 * RTR frames are defined as 'r' types - normal frames have 't' type:
 * t => 11 bit data frame
 * r => 11 bit RTR frame
 * T => 29 bit data frame
 * R => 29 bit RTR frame
 *
 * The <id> is 3 (standard) or 8 (extended) bytes in ASCII Hex (base64).
 * The <dlc> is a one byte ASCII number ('0' - '8')
 * The <data> section has at much ASCII Hex bytes as defined by the <dlc>
 *
 * Examples:
 *
 * t1230 : can_id 0x123, can_dlc 0, no data
 * t4563112233 : can_id 0x456, can_dlc 3, data 0x11 0x22 0x33
 * T12ABCDEF2AA55 : extended can_id 0x12ABCDEF, can_dlc 2, data 0xAA 0x55
 * r1230 : can_id 0x123, can_dlc 0, no data, remote transmission request
 *
 * To control canserial, we have:
 *
 * canserial command
 * c<command bytes>\n
 *
 * For commands, we use the following:
 *
 * CAN device command
 * D<command bytes>\n
 *
 * <command bytes>
 * byte 0 - command desired
 * byte 1 - length of command data
 * bytes 2-10 - command data bytes (number depends on command data)
 *
 * For errors, we use the following:
 *
 * CAN error
 * E<error bytes>\n
 *
 * <error bytes>
 * each byte is 2 hex chars
 * byte 0 = error code (see defines in canserial.h && can.h)
 * byte 1 = dev_buffer (0 if no meaning for this code)
 * byte 2 = dev_code (0 if no meaning for this code)
 *
 *
 *******************************************************************
 * canserial commands
 *******************************************************************
 * 0 - set baud rate. When this is received, the baud rate is sent
 * and all communication after this is at the new baud rate.
 * c00<dl><baud rate bytes>\n
 * 
 * <baud rate bytes>
 * the baud rate in decimal (hex digits), 
 * for example 9600 w/b '2' '5' '8' '0' (0x2580) dl:2
 */

// define to 1 if debugging
/* #define CANSERIALDEBUG 1 */

// canserial errors
#define CANSERIAL_FLAGBIT 0x80

// canserial errors
#define CANSERIAL_OK 0
#define CANSERIAL_CMD_TOO_LONG (CANSERIAL_FLAGBIT|1)
#define CANSERIAL_BAUDRATE_BAD_INPUT (CANSERIAL_FLAGBIT|2)

// some definitions
#define CAN_TRANSMIT_TIMEOUT_ERR 0x00

// structure to hold CAN serial functions and data
struct can_serial
{
	/* buffer to hold message to send over CAN */
	can_msg_t send_msg;
	/* buffer to hold can device command */
	struct can_device_command dev_cmd;

	/**** Following fn's may be 0 if not desired ****/

	/* log a message received on can bus */
	void (*can_log_recv_message)(const can_msg_t* msg);
	/* log a message sent on can bus */
	void (*can_log_send_message)(const can_msg_t* msg);
	/* log a canserial command */
	void (*can_log_serial_command)(const struct can_device_command* cmd);
	/* log a canserial failed message */
	void (*can_log_failed_serial_command)(const struct can_device_command* cmd,
		uint8_t status);
	/* log a can device message */
	void (*can_log_device_command)(const struct can_device_command* cmd);
	/* log a can device failed message */
	void (*can_log_failed_device_command)(const struct can_device_command* cmd,
		uint8_t status);

	/**** Following fn's must not be 0, always executed ****/

	/* send a message on the can bus, returns CAN_OK or CAN_FAILTX */
	int (*can_send_message)(const can_msg_t* msg);
	/* execute a can device command */
	int (*can_device_command)(const struct can_device_command* cmd);
};

// do a canserial command
extern int canserial_command(const struct can_device_command* cmd);

// parse an input buffer
extern int canserial_parse_input_buffer(struct can_serial* dev, 
										uint8_t (*count)(void),
										uint8_t (*getc)(void));

// do something with a can message received
extern int canserial_handle_recv_message(struct can_serial* dev, 
										 const can_msg_t* msg);

// logging functions
extern void log_send(const can_msg_t* msg);

extern void log_recv(const can_msg_t* msg);

extern void log_canserial_cmd(const struct can_device_command* cmd);

extern void log_failed_cmd(const struct can_device_command* cmd, 
						   uint8_t status);

extern void log_dev_cmd(const struct can_device_command* cmd);

#endif
