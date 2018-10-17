#include "defs.h"
#include <string.h>
#include "can.h"
#include "can_delays.h"

/*-----------------------------------------------------------------------*/

#undef PDEBUG
#undef PDEBUG2
#ifdef CANDEBUG

#include <stdio.h>

static const char* k_can_name = "can:";

#  ifdef __AVR
#    include <avr/pgmspace.h>

#    define PDEBUG(MSG) (printf_P(PSTR("%s" MSG "\n"), k_can_name))
#    define PDEBUG2(MSG, ...) (printf_P(PSTR("%s" MSG "\n"), k_can_name, __VA_ARGS__))

#  else

#    define PDEBUG(MSG) (printf("%s" MSG "\n", k_can_name))
#    define PDEBUG2(MSG, ...) (printf("%s" MSG "\n", k_can_name, __VA_ARGS__))

#  endif

#else

#  define PDEBUG(MSG) /* nothing */
#  define PDEBUG2(MSG, ...) /* nothing */

#endif

/*-----------------------------------------------------------------------*/

int can_init(can_init_t* settings, struct can_device* dev)
{
	PDEBUG("can init");

    dev->settings = settings;

	return dev->init_fn(settings, dev);
}

int can_reinit(struct can_device* dev)
{
	PDEBUG("can reinit");

	return dev->reinit_fn(dev);
}

int can_self_test(struct can_device *dev)
{
	PDEBUG("can self test");

	return dev->self_test_fn(dev);
}

void can_init_message_struct(can_msg_t* msg)
{
	memset(msg, 0, sizeof(can_msg_t));
}

int can_check_receive(struct can_device *dev)
{
	PDEBUG("can check receive");

	return dev->check_receive_fn(dev);
}

int can_send_message(struct can_device *dev, const can_msg_t* msg)
{
	int res;
	int txbuf_n;
	uint32_t time_ms;
	uint8_t timeout = 0;
	
	PDEBUG("can send message");

	if (dev->bus_state == BUS_OFF)
		return CAN_FAILTX;
	
 	time_ms = millis();
	
	do {
		res = dev->free_send_buffer_fn(dev, &txbuf_n);
		if ((millis() - time_ms) > dev->settings->tx_wait_ms )
			timeout = -1;
	} while (res == CAN_ALLTXBUSY && !timeout);
	if (timeout) {
		PDEBUG("can transmit timeout");
		return CAN_FAILTX;
	}
	dev->write_msg_fn(dev, txbuf_n, msg);
	return CAN_OK;
}

int can_read_message(struct can_device *dev, can_msg_t* msg)
{
	PDEBUG("can read message");
	return dev->read_msg_fn(dev, msg);
}

int can_device_command(struct can_device *dev,
                       const struct can_device_command* dev_cmd)
{
	PDEBUG("can device command");
	int errcode = CAN_OK;

	/* loopback */
	if(dev_cmd->command == CAN_LOOPBACK_ENABLE) {
		if(dev_cmd->length != CAN_LOOPBACK_ENABLE_SZ)
			errcode = CAN_FAIL;
		else if(dev_cmd->data[0] != dev->settings->loopback_on) {
			dev->settings->loopback_on = dev_cmd->data[0];
			PDEBUG("can changing loopback mode");
			errcode = can_init(dev->settings, dev);
		}
	}
	/* baudrate change */
	else if(dev_cmd->command == CAN_BAUDRATE) {
		if(dev_cmd->length != CAN_BAUDRATE_SZ)
			errcode = CAN_FAIL;
		else {
			uint8_t original_baud = dev->settings->speed_setting;
			PDEBUG("can changing baudrate");
			/* try to set the new baud rate */
			dev->settings->speed_setting = dev_cmd->data[0];
			errcode = can_init(dev->settings, dev);
			/* if that doesn't work, go back to the old baud rate */
			if(errcode == CAN_FAILINIT) {
				PDEBUG("changing baudrate doesn't work, using original baud rate");
				dev->settings->speed_setting = original_baud;
				errcode = can_init(dev->settings, dev);
			}
		}
	}
	/* tx timeout */
	else if(dev_cmd->command == CAN_TX_TIMEOUT_MS) {
		if(dev_cmd->length != CAN_TX_TIMEOUT_MS_SZ)
			errcode = CAN_FAIL;
		else {
			dev->settings->tx_wait_ms = (dev_cmd->data[0] << 8) + dev_cmd->data[1];
			PDEBUG2("can new tx timeout:%d", dev->settings->tx_wait_ms);
		}
	}

	return errcode;
}

int can_handle_interrupt(struct can_device *dev,
                         int* status_flag)
{
	return dev->handle_int_fn(dev, status_flag);
}

void can_handle_error(struct can_device *dev, const can_error_t* err)
{
    // check for changes in bus state
    if (err->error_code == CAN_BUS_PASSIVE)
        dev->bus_state = ERROR_PASSIVE;
    if (err->error_code == CAN_BUS_OFF)
        dev->bus_state = BUS_OFF;

    // call specific error function if present
	if (dev->handle_error_fn)
		dev->handle_error_fn(dev, err);
}

int can_error_counts(struct can_device *dev,
                     uint8_t* tx_count, uint8_t* rx_count)
{
	return dev->error_counts(dev, tx_count, rx_count);
}

void can_clear_tx_buffers(struct can_device *dev)
{
	dev->clear_tx_buffers(dev);
}
