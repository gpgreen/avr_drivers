#ifndef CANAERO_NIS_H_
#define CANAERO_NIS_H_

#include "canaero.h"

/*-----------------------------------------------------------------------*/

extern void canaero_get_nis_data(can_msg_t* msg);

extern int canaero_reply_nis(canaero_init_t *proto, 
                             service_msg_id_t* svc, can_msg_t* msg);

/*-----------------------------------------------------------------------*/

#endif
