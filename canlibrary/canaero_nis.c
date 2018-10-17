#include "defs.h"
#include "globals.h"
#include "canaero_nis.h"

/*-----------------------------------------------------------------------*/

void
canaero_get_nis_data(can_msg_t *msg)
{
  msg->data[3] = canaero_get_nodeid(&CAN_config);
}

/*-----------------------------------------------------------------------*/

// NIS service reply
// set a new node id
int
canaero_reply_nis(canaero_init_t *proto,
                  service_msg_id_t* svc, can_msg_t* msg)
{
  /* Node-ID Setting Service Request */
  static canaero_svc_msg_tmpl_t t = {NODATA, 11, 0, canaero_get_nis_data};
	
  /* Node-ID Setting Service Request (bad-id) */
  static canaero_svc_msg_tmpl_t bad_t = {NODATA, 11, -6, 0};
		
  int snd_stat;

  // new node id is found in message code
  uint8_t new_id = msg->data[3];
  if (new_id >= 1 && new_id <= 199) {
    canaero_set_nodeid(proto, new_id);
    snd_stat = canaero_send_svc_reply_message(proto, svc, &t);
  } else {
    snd_stat = canaero_send_svc_reply_message(proto, svc, &bad_t);
  }
  return snd_stat;
}



