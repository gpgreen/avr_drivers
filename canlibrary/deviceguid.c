#include "defs.h"

#include <avr/eeprom.h>

#include "deviceguid.h"

/*-----------------------------------------------------------------------*/

/*
  generated_uuid.i should contain the following definition

struct device_guid s_eeprom_guid EEMEM = {
    {
        0xff,0xff,0xde,0xad,0xff,0xff,0xde,0xad,
        0xff,0xff,0xde,0xad,0xff,0xff,0xde,0xad,
    },

*/
#include "generated_uuid.i"

/*-----------------------------------------------------------------------*/

// global variable holding eeprom values
struct device_guid g_guid;

void device_save_guid(void)
{
    // write the guid to eeprom
    eeprom_write_block(&g_guid, &s_eeprom_guid, sizeof(struct device_guid));
}

void device_load_guid(void)
{
    // read the guid from eeprom
    eeprom_read_block(&g_guid, &s_eeprom_guid, sizeof(struct device_guid));
}

