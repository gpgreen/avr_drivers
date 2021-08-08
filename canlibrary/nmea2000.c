#include <string.h>
#include <avr/eeprom.h>
#include "nmea2000.h"
#include "timer.h"

/*-----------------------------------------------------------------------*/

#ifdef NMEA2000DEBUG

#include <stdio.h>

static const char* k_nmea_name = "nmea2000:";

#  ifdef __AVR
#    include <avr/pgmspace.h>

#    define PDEBUG(MSG) (printf_P(PSTR("%s" MSG "\n"), k_nmea_name))
#    define PDEBUG2(MSG, ...) (printf_P(PSTR("%s" MSG "\n"), k_nmea_name, __VA_ARGS__))

#  else

#    define PDEBUG(MSG) (printf("%s" MSG "\n", k_nmea_name))
#    define PDEBUG2(MSG, ...) (printf("%s" MSG "\n", k_nmea_name, __VA_ARGS__))

#  endif

#else

#  define PDEBUG(MSG) /* nothing */
#  define PDEBUG2(MSG, ...) /* nothing */

#endif

/*-----------------------------------------------------------------------*/

struct nmea2000_eeprom 
{
    uint8_t address;
};

struct nmea2000_eeprom s_eeprom_nmea;

/*-----------------------------------------------------------------------*/

extern inline uint16_t nmea2000_pgn_hi(const can_msg_t* msg);

extern inline uint8_t nmea2000_pgn_lo(const can_msg_t* msg);

extern inline uint8_t nmea2000_src_addr(const can_msg_t* msg);

/*-----------------------------------------------------------------------*/

static int
nmea2000_address_claimed(nmea2000_dev_t* dev, uint8_t dest_addr, uint8_t src_addr)
{
    can_msg_t ac;
    can_init_message_struct(&ac);
    // construct the can message id
    // top byte is priority
    // next byte is high byte of PGN
    // next byte is destination address
    // final byte is source address
    ac.id = ((uint32_t)0x18 << 24) | ((uint32_t)60928 << 8) | ((uint32_t)dest_addr << 8) | src_addr;
    ac.idtype = CAN_EXTENDED_ID;
    // payload is NAME
    ac.length = 8;
    memcpy(ac.data, dev->name, 8);
    // now send it
    int err = can_send_message(dev->candev, &ac);
    if (err != CAN_OK)
        return err;
    // need to wait 250ms before all set
    dev->claim_start = jiffie();
    return CAN_OK;
}

/*-----------------------------------------------------------------------*/

void nmea2000_init(struct can_device* candev, nmea2000_dev_t* dev, const uint8_t* name)
{
	PDEBUG("nmea2000 init");
    dev->candev = candev;

    dev->address = NMEA2000_PREFERRED_SOURCE_ADDRESS;
    dev->name = name;
    dev->state = INIT;
    dev->claim_start = 0;
    for (int i=0; i<NMEA2000_ADDR_MAP_SZ; i++)
        dev->map[i].src_address = 254;
    nmea2000_save_settings(dev);
}

/*-----------------------------------------------------------------------*/

void nmea2000_start_claim(nmea2000_dev_t* dev)
{
    // now do the claim
    if (nmea2000_address_claimed(dev, 255, dev->address) == CAN_OK)
        dev->state = ADDRESS_CLAIM;
}

/*-----------------------------------------------------------------------*/

void nmea2000_save_settings(const nmea2000_dev_t* dev)
{
    struct nmea2000_eeprom settings;
    eeprom_read_block(&dev, &s_eeprom_nmea, sizeof(struct nmea2000_eeprom));
    if (settings.address != dev->address) {
        settings.address = dev->address;
        eeprom_write_block(&settings, &s_eeprom_nmea, sizeof(struct nmea2000_eeprom));
    }
}

/*-----------------------------------------------------------------------*/

static void
nmea2000_cannot_claim_address(nmea2000_dev_t* dev)
{
    dev->address = 254;
    dev->state = ADDRESS_CLAIMED;
    nmea2000_address_claimed(dev, 255, dev->address);
}

/*-----------------------------------------------------------------------*/

// -1 if name_a < name_b
// 0 if name_a == name_b
// 1 if name_a > name_b
static int
nmea2000_compare_name(const uint8_t* name_a, const uint8_t* name_b)
{
    for (int i=0; i<8; i++) {
        if (name_a[i] != name_b[i]) {
            if (name_a[i] < name_b[i])
                return -1;
            else
                return 1;
        }
    }
    return 0;
}

/*-----------------------------------------------------------------------*/

const uint8_t*
nmea2000_lookup_name(nmea2000_dev_t* dev, uint8_t src_addr)
{
    for (int i=0; i<NMEA2000_ADDR_MAP_SZ; i++)
        if (dev->map[i].src_address == src_addr)
            return dev->map[i].name;
    return NULL;
}

/*-----------------------------------------------------------------------*/

// maintain a map of source address's to name's
static void
nmea2000_address_map(nmea2000_dev_t* dev, const can_msg_t* msg)
{
    int unused = -1;
    uint8_t src_addr = nmea2000_src_addr(msg);
    for (int i=0; i<NMEA2000_ADDR_MAP_SZ; i++) {
        // address cannot claimed, so remove from map if there
        if (dev->map[i].src_address != 254
            && src_addr == 254
            && nmea2000_compare_name(dev->map[i].name, msg->data) == 0) {
            memset(dev->map[i].name, 0, 8);
            dev->map[i].src_address = 254;
            break;
        }
        // another device has claimed address, so update name
        if (dev->map[i].src_address == src_addr
            && nmea2000_compare_name(dev->map[i].name, msg->data) > 0) {
            memcpy(dev->map[i].name, msg->data, 8);
            break;
        }
        // track the first unused entry
        if (dev->map[i].src_address == 254 && unused == -1) {
            unused = i;
        }
    }
    // if address claim not handled in loop, add entry to map, if there is unused slot
    if (unused >= 0 && src_addr != 254) {
        dev->map[unused].src_address = src_addr;
        memcpy(dev->map[unused].name, msg->data, 8);
    }
}

/*-----------------------------------------------------------------------*/

// received an address claim message, react to it
static int
nmea2000_handle_address_claim(nmea2000_dev_t* dev, const can_msg_t* msg)
{
    if (nmea2000_src_addr(msg) == dev->address) {
        // some other device has claimed the same address
        if (nmea2000_compare_name(dev->name, msg->data) == -1) {
            // our name is less, we own the address
            nmea2000_address_claimed(dev, 255, dev->address);
        } else {
            // their name controls, go offline
            nmea2000_cannot_claim_address(dev);
        }
    } else {
        nmea2000_address_map(dev, msg);
    }
    return 1;
}

/*-----------------------------------------------------------------------*/

// received a request message, react to it
static int
nmea2000_handle_request_message(nmea2000_dev_t* dev, const can_msg_t* msg)
{
    uint8_t pgn_lo = nmea2000_pgn_lo(msg);
    uint32_t pgn = ((uint32_t)msg->data[0] << 16) | ((uint32_t)msg->data[1])
        | ((uint32_t)msg->data[2]);
    if ((pgn == 60928 && nmea2000_src_addr(msg) == 254)
        && ((pgn_lo == dev->address) || (pgn_lo == 255))) {
        // device asking for address claims, to network, or this device
        nmea2000_address_claimed(dev, 255, dev->address);
    }
    return 1;
}

/*-----------------------------------------------------------------------*/

// check each can message for network handling
int
nmea2000_can_message_received(nmea2000_dev_t* dev, const can_msg_t* msg)
{
    // extract pgn
    if (msg->idtype != CAN_EXTENDED_ID || dev->state == ADDRESS_CLAIMED || dev->state == INIT)
        return 0;
    uint16_t pgnhi = nmea2000_pgn_hi(msg);
    int handled = 0;
    while (handled == 0) {
        switch (pgnhi) {
            // address claimed
        case 60928:
            handled = nmea2000_handle_address_claim(dev, msg);
            break;
            // request message
        case 59904:
            handled = nmea2000_handle_request_message(dev, msg);
            break;
        }
    }
    return 1;
}

/*-----------------------------------------------------------------------*/

// check for timeouts, etc
void
nmea2000_periodic_handling(nmea2000_dev_t* dev)
{
    switch (dev->state) {
    case ADDRESS_CLAIM:
        if (duration(dev->claim_start) >= 250) {
            dev->state = OPERATIONAL;
            dev->claim_start = 0;
        }
        break;
    case ADDRESS_CLAIMED:
        break;
    case OPERATIONAL:
        break;
    case INIT:
        break;
    }
}

/*-----------------------------------------------------------------------*/

