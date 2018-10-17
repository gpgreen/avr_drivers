#ifndef DEVICE_GUID_H_
#define DEVICE_GUID_H_

#define DEVICE_GUID_LEN 16

// structure to hold guid in eeprom
struct device_guid
{
    uint8_t buf[DEVICE_GUID_LEN];
};

// global variable holding eeprom values
extern struct device_guid g_guid;

/*-----------------------------------------------------------------------*/

extern void device_save_guid(void);
extern void device_load_guid(void);

/*-----------------------------------------------------------------------*/

#endif
