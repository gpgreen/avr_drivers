#include "defs.h"
#include "globals.h"
#include "canaero_bss.h"

/*-----------------------------------------------------------------------*/

// BSS service reply
// set baudrate
int
canaero_reply_bss(canaero_init_t *proto, service_msg_id_t* svc, can_msg_t* msg)
{
	/* Baudrate setting service request */
	static canaero_svc_msg_tmpl_t bad_baud = {NODATA, 10, -1, 0};

	if (msg->data[1] != SHORT)
	{
		return canaero_send_svc_reply_message(proto, svc, &bad_baud);
	}
	
	uint8_t original_baud = proto->can_settings.speed_setting;
	uint16_t baud_rate = (msg->data[5] << 8) + msg->data[4];

	switch(baud_rate) {
	case 0:
		// 1 MBit/s
		proto->can_settings.speed_setting = CAN_1000KBPS;
		break;
	case 1:
		// 500 kbit/s
		proto->can_settings.speed_setting = CAN_500KBPS;
		break;
	case 2:
		// 250 kbit/s
		proto->can_settings.speed_setting = CAN_250KBPS;
		break;
	case 3:
		// 125 kbit/s
		proto->can_settings.speed_setting = CAN_125KBPS;
		break;
	case 4:
		// 20 kbit/s
		proto->can_settings.speed_setting = CAN_20KBPS;
		break;
	default:
		// only time we respond is when we have a bad baud rate
		return canaero_send_svc_reply_message(proto, svc, &bad_baud);
	}
	// try to set the new baud rate
	errcode = canaero_reinit(proto, proto->can_dev);
	// if it don't work, the set it back to the old one
	if (errcode == CAN_FAILINIT) {
		proto->can_settings.speed_setting = original_baud;
		errcode = canaero_reinit(proto, proto->can_dev);
		// for some reason, setting the old one failed also
		if (errcode == CAN_FAILINIT)
			return errcode;
		return canaero_send_svc_reply_message(proto, svc, &bad_baud);
	}
	// we've set the new baud rate
	return CAN_OK;
}

	
