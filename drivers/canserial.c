#include <stdio.h>
#include <string.h>
#include "canserial.h"
#include "uart.h"

#ifdef CAN_TIMESTAMP
#include "timer.h"
#endif

/* handles a received message, does logging */
int canserial_handle_recv_message(struct can_serial* dev, const can_msg_t* msg)
{
	if(dev->can_log_recv_message)
		dev->can_log_recv_message(msg);
	return CAN_OK;
}

/* canserial command */
int canserial_command(const struct can_device_command* cmd)
{
	if (cmd->length > CAN_DEVICE_CMD_MAX_DATA)
		return CANSERIAL_CMD_TOO_LONG;
	if (cmd->command == 0) {
		if (cmd->length > 3)
			return CANSERIAL_BAUDRATE_BAD_INPUT;
		uint8_t ubrrh = cmd->data[0];
		uint8_t ubrrl = cmd->data[1];
		uint8_t use_2x = cmd->data[2];
		uart_set_baudrate(ubrrh, ubrrl, use_2x);
	}
	return CANSERIAL_OK;
}

static void print_nibble(uint8_t nibble)
{
	if (nibble >= 10)
		putchar(nibble - 10 + 'A');
	else
		putchar(nibble + '0');
}

static void print_byte(uint8_t byte)
{
	print_nibble((byte & 0xf0) >> 4);
	print_nibble((byte & 0xf));
}

union msg_id_bytes 
{
	uint32_t id;
	uint8_t bytes[4];
};

/* print out a can message to stdout */
static void print_msg(const can_msg_t* msg)
{
	union msg_id_bytes mid;
	mid.id = msg->id;
	/* the id */
    if (msg->idtype == CAN_EXTENDED_ID)
    {
        print_byte(mid.bytes[3]);
        print_byte(mid.bytes[2]);
        print_byte(mid.bytes[1]);
        print_byte(mid.bytes[0]);
    } else {
        print_nibble(mid.bytes[1]);
        print_nibble(mid.bytes[0] >> 4);
        print_nibble(mid.bytes[0] & 0xf);
    }
    
	/* the length */
	putchar('0' + msg->length);
	/* the data */
	for(int i=0; i<msg->length; ++i)
		print_byte(msg->data[i]);
}

/* log a send can message - default implementation */
void log_send(const can_msg_t* msg)
{
#ifdef CAN_TIMESTAMP
    printf("-> <%ld>", msg->tstamp);
#else
    putchar('=');
#endif
	putchar(msg->idtype == CAN_EXTENDED_ID ? 'T' : 't');
	print_msg(msg);
	putchar('\n');
}

/* log a received can message - default implementation */
void log_recv(const can_msg_t* msg)
{
#ifdef CAN_TIMESTAMP
    /* timestamp in jiffies */
    printf("<%ld>", msg->tstamp);
#endif
	/* received can message */
	putchar(msg->idtype == CAN_EXTENDED_ID ? 'T' : 't');
	print_msg(msg);
	putchar('\n');
}

void log_canserial_cmd(const struct can_device_command* cmd)
{
	putchar('=');
	putchar('c');
	print_byte(cmd->command);
	print_byte(cmd->length);
	for(int i=0; i<cmd->length; ++i)
		print_byte(cmd->data[i]);
	putchar('\n');
}

void log_failed_cmd(const struct can_device_command* cmd,
	uint8_t status)
{
	putchar('E');
	print_byte(status);
	putchar('\n');
}

void log_dev_cmd(const struct can_device_command* cmd)
{
	putchar('=');
	putchar('D');
	print_byte(cmd->command);
	print_byte(cmd->length);
	for(int i=0; i<cmd->length; ++i)
		print_byte(cmd->data[i]);
	putchar('\n');
}

static int parse_data_hexdecimal(uint8_t ch, uint8_t* val)
{
	if (ch >= 'a' && ch <= 'f')
		*val = (*val << 4) | (ch - 'a' + 10);
	else if (ch >= 'A' && ch <= 'F')
		*val = (*val << 4) | (ch - 'A' + 10);
	else if (ch >= '0' && ch <= '9')
		*val = (*val << 4) | (ch - '0');
	else
		return -1;
	return 0;
}

static int parse_id_hexdecimal(uint8_t ch, uint32_t* val)
{
	if (ch >= 'a' && ch <= 'f')
		*val = (*val << 4) | (ch - 'a' + 10);
	else if (ch >= 'A' && ch <= 'F')
		*val = (*val << 4) | (ch - 'A' + 10);
	else if (ch >= '0' && ch <= '9')
		*val = (*val << 4) | (ch - '0');
	else
		return -1;
	return 0;
}

/* parse_input_buffer
 *
 * returns number of bytes in buffer parsed
 *
 * Look for the constructs in the input buffer, execute them
 */
int canserial_parse_input_buffer(struct can_serial* dev, struct fifo* ififo)
{
	int parse_state = 0;
	int hex_chars_to_go = 0;
	int msg_seq = 0;
	int consumed = 0;
	int serial_command = 0;
	
	// iterate through the buffer
	int i=0;
	while(fifo_count(ififo)) {
		uint8_t ch;
        if(fifo_get_unsafe(ififo, &ch) == FIFO_EMPTY)
            break;
		i++;
#ifdef CANSERIALDEBUG
		printf("i:%d ch:0x%x p:%d hc:%d m:%d c:%d\n", i, (unsigned char)ch,
			   parse_state, hex_chars_to_go, msg_seq, consumed);
#endif
		switch(parse_state) {
		case 0:
			msg_seq = 0;
			serial_command = 0;
			if(ch == 't' || ch == 'r') {
                // standard id CAN message
				hex_chars_to_go = 3;
				parse_state = 1;
				memset(&dev->send_msg, 0, sizeof(can_msg_t));
#ifdef CAN_TIMESTAMP
                dev->send_msg.tstamp = jiffie();
#endif
                dev->send_msg.idtype = CAN_STANDARD_ID;
				dev->send_msg.rtr = (ch =='r') ? 1 : 0;
			} else if(ch == 'T' || ch == 'R') {
                // extended id CAN message
				hex_chars_to_go = 8;
				parse_state = 1;
				memset(&dev->send_msg, 0, sizeof(can_msg_t));
#ifdef CAN_TIMESTAMP
                dev->send_msg.tstamp = jiffie();
#endif
                dev->send_msg.idtype = CAN_EXTENDED_ID;
				dev->send_msg.rtr = (ch =='R') ? 1 : 0;
			} else if(ch == 'D' || ch == 'c') {
				hex_chars_to_go = 2;
				parse_state = 4;
				memset(&dev->dev_cmd, 0, sizeof(struct can_device_command));
				if (ch == 'c')
					serial_command = 1;
			}
			// any other character is skipped
			else
				consumed = i;
			break;
		case 1:					/* message id */
			if(parse_id_hexdecimal(ch, &(dev->send_msg.id)) == 0) {
				if (--hex_chars_to_go == 0)
					parse_state = 2;
			} else
				parse_state = 0;
			break;
		case 2:					/* dlc */
			if(ch >= '0' && ch <= '8') {
				dev->send_msg.length = (ch - '0');
				parse_state = 3;
				hex_chars_to_go = dev->send_msg.length * 2;
				msg_seq = 0;
				if (hex_chars_to_go == 0) {
					// got the whole message, send it..
					// send the message...
					consumed = i;
					if(dev->can_log_send_message != 0)
						dev->can_log_send_message(&dev->send_msg);
					uint8_t status = dev->can_send_message(
                        dev->candev, &dev->send_msg);
					if (status == CAN_FAILTX) {
						can_error_t e;
						e.error_code = CAN_FAILTX;
						e.dev_buffer = 0;
						e.dev_code = 0;
						dev->can_handle_error(dev->candev, &e);
					}
					parse_state = 0;
				}
			}
			else
				parse_state = 0;
			break;
		case 3:					/* data */
			if(parse_data_hexdecimal(ch, &(dev->send_msg.data[msg_seq])) == 0) 
			{
				if (--hex_chars_to_go == 0) {
					// got the whole message, send it..
					// send the message...
					consumed = i;
					if(dev->can_log_send_message != 0)
						dev->can_log_send_message(&dev->send_msg);
					uint8_t status = dev->can_send_message(
                        dev->candev, &dev->send_msg);
					if (status == CAN_FAILTX) {
						can_error_t e;
						e.error_code = CAN_FAILTX;
						e.dev_buffer = 0;
						e.dev_code = 0;
						dev->can_handle_error(dev->candev, &e);
					}
					parse_state = 0;
				} else if ((hex_chars_to_go & 0x1) == 0)
					++msg_seq;
			} else
				parse_state = 0;
			break;
		case 4:					/* start of canserial/device command command byte */
			if(parse_data_hexdecimal(ch, &(dev->dev_cmd.command)) == 0) {
				if (--hex_chars_to_go == 0) {
					parse_state = 5;
					hex_chars_to_go = 2;
				}
			}
			else
				parse_state = 0;
			break;
		case 5:					/* canserial/device command data length */
			if(parse_data_hexdecimal(ch, &(dev->dev_cmd.length)) == 0) {
				if (--hex_chars_to_go == 0) {
					parse_state = 6;
					hex_chars_to_go = dev->dev_cmd.length * 2;
					msg_seq = 0;
				}
			} else
				parse_state = 0;
			break;
		case 6:					/* canserial/device command data bytes */
			if(parse_data_hexdecimal(ch, &(dev->dev_cmd.data[msg_seq])) == 0) {
				if (--hex_chars_to_go == 0) {
					consumed = i;
					// got the whole command, execute it
					if (!serial_command) {
						if (dev->can_log_device_command != 0)
							dev->can_log_device_command(&dev->dev_cmd);
						uint8_t status = dev->can_device_command(
                            dev->candev, &dev->dev_cmd);
						if (status != CAN_OK
							&& dev->can_log_failed_device_command != 0)
							dev->can_log_failed_device_command(&dev->dev_cmd,
															   status);
					} else {
						if (dev->can_log_serial_command != 0)
							dev->can_log_serial_command(&dev->dev_cmd);
						uint8_t status = canserial_command(&dev->dev_cmd);
						if (status != CANSERIAL_OK
							&& dev->can_log_failed_serial_command != 0)
							dev->can_log_failed_serial_command(&dev->dev_cmd,
															   status);
					}
					parse_state = 0;
				} else if ((hex_chars_to_go & 0x1) == 0)
					++msg_seq;
			}
			else
				parse_state = 0;
			break;
		}
	}
	return consumed;
}
