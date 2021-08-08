#ifndef NMEA2000_H_
#define NMEA2000_H_

#include "defs.h"
#include <stdint.h>
#include "can.h"

/*-----------------------------------------------------------------------*/

// uncomment for debugging
//#define NMEA2000DEBUG       1

// the following must be defined in defs.h for size of the source address
// to NAME size
#ifndef NMEA2000_ADDR_MAP_SZ
#error "This driver requires a NMEA2000_ADDR_MAP_SZ define for length of map"
#endif

// the following must be defined in defs.h for initial source address claims
// it is an 8bit unsigned int, can't use 254 or 255
#ifndef NMEA2000_PREFERRED_SOURCE_ADDRESS
#error "This driver requires a NMEA2000_PREFERRED_SOURCE_ADDRESS define for 8bit preferred source address"
#endif

/*-----------------------------------------------------------------------*/

typedef enum nmea2000_state {
    INIT = 0,
    ADDRESS_CLAIM,
    ADDRESS_CLAIMED,
    OPERATIONAL,
} nmea2000_state_t;

/*-----------------------------------------------------------------------*/

typedef struct nmea2000_addr_map 
{
    uint8_t src_address;
    uint8_t name[8];
} nmea2000_addr_map_t;

/*-----------------------------------------------------------------------*/

// hardware specific functions
typedef struct nmea2000_device
{
    struct can_device* candev;
    uint8_t address;
    const uint8_t* name;
    nmea2000_state_t state;
    uint32_t claim_start;
    nmea2000_addr_map_t map[NMEA2000_ADDR_MAP_SZ];
} nmea2000_dev_t;

/*-----------------------------------------------------------------------*/

// driver functions

extern void nmea2000_init(
    struct can_device* candev, nmea2000_dev_t* dev, const uint8_t* name);

extern void nmea2000_save_settings(const nmea2000_dev_t* dev);

extern void nmea2000_start_claim(nmea2000_dev_t* dev);

extern int nmea2000_can_message_received(nmea2000_dev_t* dev, const can_msg_t* msg);

extern const uint8_t* nmea2000_lookup_name(nmea2000_dev_t* dev, uint8_t src_addr);

extern void nmea2000_periodic_handling(nmea2000_dev_t* dev);

// utility functions

inline uint16_t nmea2000_pgn_hi(const can_msg_t* msg)
{
    return (uint16_t)((msg->id & 0xFF0000) >> 8);
}

inline uint8_t nmea2000_pgn_lo(const can_msg_t* msg)
{
    return (uint8_t)((msg->id & 0xFF00) >> 8);
}

inline uint8_t nmea2000_src_addr(const can_msg_t* msg)
{
    return (uint8_t)(msg->id & 0xFF);
}

/*-----------------------------------------------------------------------*/

#endif // NMEA2000_H_
