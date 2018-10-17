#ifndef CANAERO_IDS_H_
#define CANAERO_IDS_H_

#include "canaero.h"

/*-----------------------------------------------------------------------*/

extern void canaero_get_ids_data(can_msg_t* msg);

extern int canaero_reply_ids(canaero_init_t *proto, 
                             service_msg_id_t* svc, can_msg_t* msg);

/*-----------------------------------------------------------------------*/

#endif
