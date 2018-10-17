#include "defs.h"
#include "globals.h"
#include "canaero_nis.h"

/*-----------------------------------------------------------------------*/

void
canaero_get_ids_data(can_msg_t *msg)
{
	msg->data[4] = HARDWARE_REVISION;
	msg->data[5] = SOFTWARE_REVISION;
	msg->data[6] = 0;			/* CAN-aerospace standard distribution */
	msg->data[7] = 0;			/* CAN-aerospace standard header */
}

/*-----------------------------------------------------------------------*/

// IDS service reply
// this is for sign-of-life
uint8_t
canaero_reply_ids(canaero_init_t *proto, service_msg_id_t* svc,
                  can_msg_t* msg)
{
	/* 0 Identification Service Request */
	static canaero_svc_msg_tmpl_t t = {UCHAR4, 0, 0, canaero_get_ids_data};

	// the outgoing message code matches the incoming message code
	t.msg_code = msg->data[3];
	return canaero_send_svc_reply_message(proto, svc, &t);
}




