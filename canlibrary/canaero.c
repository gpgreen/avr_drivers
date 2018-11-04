#include "canaero.h"
#include "globals.h"

/*---------------------------------------------------------------------------*/
#undef PDEBUG
#undef PDEBUG2
#ifdef CANAERODEBUG

#include <stdio.h>
#include "conversion.h"
#include "watchdog.h"

// name of driver to use in debugging printf's
static const char* k_canaero_name = "canaero:";

#  ifdef __AVR

#    include <avr/pgmspace.h>
#    define PDEBUG(MSG) (printf_P(PSTR("%s" MSG "\n"), k_canaero_name))
#    define PDEBUG2(MSG, ...) (printf_P(PSTR("%s" MSG "\n"), k_canaero_name, __VA_ARGS__))

#  else

#    define PDEBUG(MSG) (printf("%s" MSG "\n", k_canaero_name))
#    define PDEBUG2(MSG, ...) (printf("%s" MSG "\n", k_canaero_name, __VA_ARGS__))

#  endif

#else

#  define PDEBUG(MSG) /* nothing */
#  define PDEBUG2(MSG, ...) /* nothing */

#endif
/*---------------------------------------------------------------------------*/

/*
 * Message acceptance filters
 * --------------------------
 * so that device isn't swamped by received CAN messages, we
 * can apply acceptance filters. The normal filter would be
 * to only accept high priority node service data directed
 * to this node (or broadcast). This means only messages between
 * 0x80 and 0xc7, with a node id of 0 (broadcast) or node id
 * as set in the stack
 *
 * To start we will assume that filters are as done on the mcp2515,
 * with a mask, and up to 5 filters. The mask criteria is all bits
 * that are 0 are not checked (passed). Any bit that is 1, is
 * compared with all of the filters to see if it should be
 * accepted.  So if we want to only check for high priority node
 * service messages. A further note is that we only want to accept
 * requests on the channel, not replies. So we can discard all
 * messages with the lower bit set which is a reply.
 *
 * Masks and filters to accept high priority node service messages
 * for node 1 and broadcast. Also takes some user defined high
 * priority messages since these ids overlap the filter.
 *
 * standard id bits
 * 10 9 8 7 6 5 4 3 2 1 0
 * ----------------------
 *  1 1 1 1 1 1 1 1 1 1 1      Mask
 *  0 0 0 1 1 1 1 1 1 1 0      Filter 1, accepts ids without lwr bit set (0 - 0xfc)
 *  0 0 0 1 1 1 1 1 1 1 0      Filter 2, accepts ids without lwr bit set (0 - 0xfc)
 *
 * extended id bits
 * 17 16 15 14 13 12 11 10 9 8 7 6 5 4 3 2 1 0
 * -------------------------------------------
 *  x  x  1  1  1  1  1  1 1 1 0 0 0 0 0 0 0 0    Mask
 *  x  x  0  0  0  0  0  0 0 1 0 0 0 0 0 0 0 0    Filter 1 (accept node 1 only)
 *  x  x  0  0  0  0  0  0 0 0 0 0 0 0 0 0 0 0    Filter 2 (accept broadcast)
 *
 *
 */

/*-----------------------------------------------------------------------*/

// init
int canaero_init(canaero_init_t* proto, struct can_device* dev)
{
    PDEBUG("init");

    proto->can_dev = dev;
    
    canaero_set_nodeid(proto, proto->node_id);

    proto->idle_fn = 0;
    proto->idle_task_user_data = 0;
    
    // print out node id and settings
    PDEBUG2("node_id:%d", proto->node_id);

    return can_init(&proto->can_settings, proto->can_dev);
}

// init
int canaero_reinit(canaero_init_t* proto, struct can_device* dev)
{
    PDEBUG("reinit");

    canaero_set_nodeid(proto, proto->node_id);

    proto->idle_fn = 0;
    proto->idle_task_user_data = 0;
    
    // print out node id and settings
    PDEBUG2("node_id:%d", proto->node_id);

    return can_reinit(proto->can_dev);
}

// self test
int canaero_self_test(canaero_init_t *proto)
{
    return can_self_test(proto->can_dev);
}

// handle CAN interrupt
int canaero_handle_interrupt(canaero_init_t *proto)
{
    int status;
    // clear the interrupts for the can stack
    return can_handle_interrupt(proto->can_dev, &status);
}

// canaero idle work function
void canaero_idle_task(canaero_init_t *proto)
{
    if(proto->idle_fn != 0) {
        PDEBUG("idle task execute");
        if(proto->idle_fn(proto, proto->idle_task_user_data)) {
            proto->idle_fn = 0;
            proto->idle_task_user_data = 0;
        }
    }
}

// canaero set idle task
void canaero_set_idle_task(canaero_init_t *proto,
                           canaero_idle_work_fn* fn, void* userdata)
{
    if(proto->idle_fn != 0) {
        PDEBUG("*** FIXME *** trying to set idle task fn when already set");
        return;
    }
    proto->idle_fn = fn;
    proto->idle_task_user_data = userdata;
    PDEBUG("set idle task");
}

int canaero_send_message(canaero_init_t *proto, canaero_msg_tmpl_t* tmpl)
{
    // copy the data into the msg buffer
    proto->outgoing.id = tmpl->id;
    proto->outgoing.rtr = tmpl->rtr;
    proto->outgoing.length = 4 + canaero_data_len(tmpl->data_type);
    proto->outgoing.data[0] = proto->node_id;
    proto->outgoing.data[1] = tmpl->data_type;
    proto->outgoing.data[2] = tmpl->svc_code;
    proto->outgoing.data[3] = tmpl->msg_code;
    // message code in NOD message is a sequence number, increment it
    if (tmpl->msgty == NOD)
        ++(tmpl->msg_code);
    // if the template has a data function, run it, this will put
    // the data into the message
    if (tmpl->db_fn != 0)
        tmpl->db_fn(&proto->outgoing);

    // send it on it's way
    int retval = can_send_message(proto->can_dev, &proto->outgoing);
    if (retval != CAN_OK)
        PDEBUG2("failed to send msg id:%lx", proto->outgoing.id);
    PDEBUG2("sent message:%lx", proto->outgoing.id);
    return retval;
}

int canaero_send_svc_reply_message(canaero_init_t *proto,
                                   service_msg_id_t* svcid,
                                   canaero_svc_msg_tmpl_t* tmpl)
{
    // copy the data into the msg buffer
    proto->outgoing.id = (svcid->priority ?
                          CANAERO_LO_SVC_START :
                          CANAERO_HI_SVC_START) + svcid->channel * 2 + 1;
    proto->outgoing.rtr = 0;
    proto->outgoing.length = 4 + canaero_data_len(tmpl->data_type);
    proto->outgoing.data[0] = proto->node_id;
    proto->outgoing.data[1] = tmpl->data_type;
    proto->outgoing.data[2] = tmpl->svc_code;
    proto->outgoing.data[3] = tmpl->msg_code;
    // if the template has a data function, run it, this will put
    // the data into the message
    if (tmpl->db_fn != 0)
        tmpl->db_fn(&proto->outgoing);

    // send it on it's way
    int retval = can_send_message(proto->can_dev, &proto->outgoing);
    PDEBUG2("send svc reply msg:%lx", proto->outgoing.id);
    return retval;
}

// check to see if message is an emergency event,
// if not return 0, if it is, return -1.
// Fill out event structure
int canaero_get_emerg_event_id(const can_msg_t* msg,
                               struct emerg_event* ee)
{
    if (msg->id > CANAERO_EMERG_END)
        return 0;
    if (msg->length < 8)
        return 0;
    ee->id = msg->id;
    ee->node = msg->data[0];
    ee->error_code = (msg->data[4] << 8) + msg->data[5];
    ee->operation_id = msg->data[6];
    ee->location_id = msg->data[7];
    return -1;
}

// send an emergency event,
// return result from can_send_message
int canaero_send_emergency_event(canaero_init_t *proto,
                                 uint8_t msg_id,
                                 int16_t error_code,
                                 int8_t operation_id,
                                 int8_t location_id)
{
    can_msg_t *msg = &proto->outgoing;
    msg->id = msg_id;
    msg->length = 8;
    msg->data[0] = proto->node_id;
    msg->data[1] = ERROR;
    msg->data[2] = 0;
    msg->data[3] = 0;
    msg->data[4] = (uint8_t)((error_code & 0xFF00) >> 8);
    msg->data[5] = (uint8_t)(error_code & 0xFF);
    msg->data[6] = operation_id;
    msg->data[7] = location_id;

    return can_send_message(proto->can_dev, msg);
}

// check to see if message is a service request,
// return 0 if not a service request
// return -1 if a service request, and fill out the values in the passed
// in struct
int canaero_get_service_msg_request_id(const can_msg_t* msg,
                                       service_msg_id_t* t)
{
    // if low bit is set, then cannot be a service channel request
    if ((msg->id & 0x1) == 0x1)
        return 0;
    // is it a high priority node service channel
    if (msg->id >= CANAERO_HI_SVC_START && msg->id <= CANAERO_HI_SVC_END) {
        t->channel = (msg->id - CANAERO_HI_SVC_START) / 2;
        t->priority = 0;
        return -1;
    }
    // or a low priority node service channel
    else if (msg->id >= CANAERO_LO_SVC_START && msg->id <= CANAERO_LO_SVC_END) {
        t->channel = (msg->id - CANAERO_LO_SVC_START) / 2;
        t->priority = 1;
        return -1;
    }
    return 0;
}

void canaero_send_messages(canaero_init_t *proto, int start, int end)
{
    if (start > proto->num_nod_templates
        || end > proto->num_nod_templates) {
        PDEBUG("send_messages: *OUT OF BOUNDS* FIXME FIXME");
        return;
    }
    for (int i=start; i<end; ++i) {
        canaero_msg_tmpl_t* tmpl = &(proto->nod_msg_templates[i]);
        // send it on it's way
        int status = canaero_send_message(proto, tmpl);
        // TODO: do something better than this
        if (status == CAN_FAILTX) {
            break;
        }
    }
}

void canaero_poll_messages(canaero_init_t *proto)
{
    // nothing here, bail
    if (can_check_receive(proto->can_dev) != CAN_MSGAVAIL)
        return;

    PDEBUG("poll start");
	
    can_init_message_struct(&proto->incoming);
	
    // get it
    if (can_read_message(proto->can_dev, &proto->incoming) == CAN_NOMSG)
        return;
    PDEBUG2("id:%lx ty:%d dl:%d", proto->incoming.id, proto->incoming.idtype, proto->incoming.length);
    for (int i=0; i<proto->incoming.length; i++)
        PDEBUG2("byte(%d):%x", i, proto->incoming.data[i]);
	
    // check for emergency event
    struct emerg_event ee;
    if (canaero_get_emerg_event_id(&proto->incoming, &ee)) {
        PDEBUG("emergency event");
        if (proto->emergency_event_fn)
            proto->emergency_event_fn(&ee);
        goto serviced;
    }
	
    // check to see if this is a service request
    // we only know about services up to code 15
    service_msg_id_t svc_msg_id;
    if (canaero_get_service_msg_request_id(&proto->incoming, &svc_msg_id)) {
        PDEBUG("service message");
        // check for zero-length message
        if (proto->incoming.length < 4) {
            PDEBUG("Message not long enough for CANAero");
            goto noservice;
        }

        PDEBUG2("node:%u", proto->incoming.data[0]);
        // node id = 0 broadcast or node id matches our own ?
        if ((proto->incoming.data[0] == 0
             || proto->incoming.data[0] == proto->node_id)
            // service code between 0 and 15
            && proto->incoming.data[2] >= 0
            && proto->incoming.data[2] <= 15) {
            PDEBUG2("svc msg(%u-%u)", proto->incoming.data[2],
                    proto->incoming.data[3]);
            // service code has type of service request
            reply_svc_fn* reply_fn = proto->nsl_dispatcher_fn_array[proto->incoming.data[2]];
            // pointer will be 0 if we don't handle this service
            if (reply_fn == 0) {
                PDEBUG("No reply fn");
                goto serviced;
            }
            // execute the reply function
            int snd_stat = reply_fn(proto, &svc_msg_id, &proto->incoming);
            if (snd_stat == CAN_FAILTX) {
                // all send buffers are busy, wait and retry
                PDEBUG("busy, no reply msg");
            }
        }
        goto serviced;
    }
noservice:
    // take incoming message and send to dispatcher if not handled above
    if (proto->incoming_msg_dispatcher_fn != 0) {
        PDEBUG("in dispatch");
        proto->incoming_msg_dispatcher_fn(&proto->incoming);
    }
serviced:
    // message was handled in service handler (or ignored because we don't do
    // that service message
    PDEBUG("poll finish");
}

uint8_t canaero_get_nodeid(canaero_init_t *proto)
{
    return proto->node_id;
}

void canaero_set_nodeid(canaero_init_t *proto, uint8_t newid)
{
    proto->node_id = newid;
}

uint32_t canaero_hi_svc_channel_to_id(uint8_t channel)
{
    return channel * 2 + CANAERO_HI_SVC_START;
}

void canaero_reset_nod_message_sequence(canaero_init_t *proto)
{
    for (int i=0; i<proto->num_nod_templates; ++i)
        proto->nod_msg_templates[i].msg_code = 0;
}

// put resets in can message
void watchdog_mis2_data(can_msg_t* msg)
{
	convert_ushort_to_big_endian(g_reset_count.watchdog_resets, &(msg->data[4]));
	convert_ushort_to_big_endian(g_reset_count.brown_out_resets, &(msg->data[6]));
}

// put resets in can message
void watchdog_mis3_data(can_msg_t* msg)
{
	convert_ushort_to_big_endian(g_reset_count.external_resets, &(msg->data[4]));
	convert_ushort_to_big_endian(g_reset_count.power_on_resets, &(msg->data[6]));
}

