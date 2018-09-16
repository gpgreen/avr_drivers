#ifndef SCP1000_DEF_H_
#define SCP1000_DEF_H_

// registers
#define SCP_REVID 0x0
#define SCP_DATAWR 0x1
#define SCP_ADDPTR 0x2
#define SCP_OPERATION 0x3
#define SCP_OPSTATUS 0x4
#define SCP_RSTR 0x6
#define SCP_STATUS 0x7
#define SCP_DATARD8 0x1f
#define SCP_DATARD16 0x20
#define SCP_TEMPOUT 0x21

// indirect registers
#define SCP_CFG 0x0
#define SCP_TWIADD 0x05

// EEPROM
#define SCP_USERDATA1 0x29
#define SCP_USERDATA2 0x2a
#define SCP_USERDATA3 0x2b
#define SCP_USERDATA4 0x2c

// OPSTATUS
#define SCP_P_OPSTATUS 0

// STATUS
#define SCP_P_STARTUP 0
#define SCP_P_RTERR 4
#define SCP_P_DRDY 5
#define SCP_P_EXT_TRIGGED 6

// CFG
#define SCP_17BIT_RES 0x05
#define SCP_15BIT_RES 0x0d

// OPERATIONS
#define SCP_OP_CANCEL_OP 0x00
#define SCP_OP_READ_INDIRECT 0x01
#define SCP_OP_WRITE_INDIRECT 0x02
#define SCP_OP_READ_EEPROM 0x05
#define SCP_OP_WRITE_EEPROM 0x06
#define SCP_OP_INIT 0x07
#define SCP_OP_HI_SPD_MODE 0x09
#define SCP_OP_HI_RES_MODE 0x0a
#define SCP_OP_ULTRA_LO_PWR_MODE 0x0b
#define SCP_OP_LOW_PWR_MODE 0x0c
#define SCP_OP_SELF_TEST 0x0f

#endif  // SCP1000_DEF_H_
