#ifndef CANAERO_H_
#define CANAERO_H_

#include "defs.h"
#include <stdint.h>
#include "can.h"
#include "canaero_defs.h"

// define to 1 if debugging
/* #define CANAERODEBUG 1 */

// definitions of emergency event error codes
#define DISPLAY_BUFFER_OVERFLOW 1

struct canaero_init_t;

struct emerg_event
{
  uint8_t id;
  uint8_t node;
  int16_t error_code;
  int8_t operation_id;
  int8_t location_id;
};

// function prototype for emergency event functions
typedef void handle_emergency_event_fn(const struct emerg_event* ee);

// structure to decipher channel and priority from service request id's
typedef struct 
{
  uint8_t channel;			/* the service channel number */
  uint8_t priority;			/* 0 = high, 1 = low */
} service_msg_id_t;

// function prototype for data functions
typedef void get_data_bytes_fn(can_msg_t *msg);

// function prototype for service message reply function
typedef int reply_svc_fn(struct canaero_init_t *proto, 
                         service_msg_id_t* svc, can_msg_t* msg);

// function prototype for received nod messages
typedef void get_incoming_msg_fn(can_msg_t *msg);

// function prototype for canaero idle work tasks
typedef int canaero_idle_work_fn(struct canaero_init_t *proto, 
                                 void* userdata);

// template for CAN Aerospace messages
typedef struct
{
  enum CanAeroMsgType msgty;
  uint32_t id;
  uint8_t rtr;
  uint8_t data_type;
  uint8_t svc_code;
  uint8_t msg_code;
  get_data_bytes_fn *db_fn;
} canaero_msg_tmpl_t;

// template for CAN Aerospace Service messages
typedef struct
{
  uint8_t data_type;
  uint8_t svc_code;
  uint8_t msg_code;
  get_data_bytes_fn *db_fn;
} canaero_svc_msg_tmpl_t;

// function to see if emergency event message
extern int canaero_get_emerg_event_id(const can_msg_t* msg,
				      struct emerg_event* ee);

// function to find channel and priority for a service request id 
extern int canaero_get_service_msg_request_id(const can_msg_t* msg,
					      service_msg_id_t* t);

// function to convert a channel to a high service can id
extern uint32_t canaero_hi_svc_channel_to_id(uint8_t channel);

// structure to hold CAN Aerospace stack functions and data
typedef struct canaero_init_t
{
  // can settings
  can_init_t can_settings;
	
  // the node id
  uint8_t node_id;

  // which service channel this device will use
  uint8_t svc_channel;

  // templates for all normal operating data sent
  canaero_msg_tmpl_t* nod_msg_templates;

  // number of templates above
  int num_nod_templates;

  // array of functions to execute when service message is received
  // index of the array, is the service code of the message
  reply_svc_fn** nsl_dispatcher_fn_array;

  // function to call for every received message
  get_incoming_msg_fn* incoming_msg_dispatcher_fn;

  // function to call for emergency event messages
  handle_emergency_event_fn* emergency_event_fn;

  // the can device associated with this protocol stack
  struct can_device* can_dev;

  // idle function currently associated with this stack
  canaero_idle_work_fn *idle_fn;
    
  // idle task user data pointer
  void *idle_task_user_data;

  // outgoing message
  can_msg_t outgoing;
  // incoming message
  can_msg_t incoming;
    
} canaero_init_t;
		
extern int canaero_init(canaero_init_t* settings, struct can_device* dev);

extern int canaero_reinit(canaero_init_t* settings, struct can_device* dev);

extern int canaero_self_test(canaero_init_t *proto);

extern int canaero_send_message(canaero_init_t *proto,
                                canaero_msg_tmpl_t* tmpl);

extern int canaero_send_svc_reply_message(canaero_init_t *proto,
                                          service_msg_id_t* svcid,
                                          canaero_svc_msg_tmpl_t* tmpl);

extern int canaero_send_emergency_event(canaero_init_t *proto,
					uint8_t msg_id,
					int16_t error_code,
					int8_t operation_id,
					int8_t location_id);


extern void canaero_send_messages(canaero_init_t *proto, 
                                  int start, int end);

extern void canaero_poll_messages(canaero_init_t *proto);

extern int canaero_handle_interrupt(canaero_init_t *proto);

// function to call when processor is idle, to handle any outstanding canaero work
extern void canaero_idle_task(canaero_init_t *proto);

// function to set idle task function
// the idle task function should return 1 if it is done executing, so that it is removed
extern void canaero_set_idle_task(canaero_init_t *proto, 
                                  canaero_idle_work_fn *fn, 
                                  void* userdata);

extern uint8_t canaero_get_nodeid(canaero_init_t *proto);

extern void canaero_set_nodeid(canaero_init_t *proto, uint8_t newid);

extern void canaero_reset_nod_message_sequence(canaero_init_t *proto);

// callback functions to load resets into a can message
extern void watchdog_mis2_data(can_msg_t* msg);
extern void watchdog_mis3_data(can_msg_t* msg);

#endif
