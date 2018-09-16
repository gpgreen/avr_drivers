/*
 * to find all the register and bit definitions look in:
 * /usr/lib/avr/include/avr/iocanxx.h
 */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <util/delay_basic.h>
#include <util/atomic.h>
#include <string.h>
#include "at90can.h"
#include "at90can_def.h"
#include "globals.h"
#include "driver.h"

/* globals */
static const char* k_name = "at90can:";

// record interrupt state
struct at90can_priv 
{
	// set bit to 1 flag interrupt reasons
	// 7 - Receive complete
	// 6 - Transmit complete
	// 5 - Bit error
	// 4 - Stuff error
	// 3 - CRC error
	// 2 - Frame error
	// 1 - Ack error
	// 0 - Bus off
	volatile uint8_t flag;

    /* the first available tx mob. All the lower number mobs
	 * are reserved for reception. All mobs starting at this
	 * index are reserved for transmission
	 */
	uint8_t first_tx_mob;

    /* keep a pointer to the settings so we can restore filters */
	can_init_t* settings;

};

// the device private struct
static struct at90can_priv s_at90can_dev_priv;

struct can_device at90can_dev = {
	.bus_state = ERROR_ACTIVE,
	.device_name = "at90can",
	.init_fn = at90can_init,
	.self_test_fn = at90can_self_test,
	.check_receive_fn = at90can_check_msg_received,
	.free_send_buffer_fn = at90can_get_next_free_tx_mob,
	.write_msg_fn = at90can_write_msg,
	.read_msg_fn = at90can_read_msg,
	.handle_int_fn = at90can_handle_interrupt,
	.handle_error_fn = 0,
	.error_counts = at90can_error_counts,
	.clear_tx_buffers = at90can_clear_tx_buffers,
};

/* These bytes record which mobs have
   transmitted messages. The bit
   will be set for each mob that has
   message ready for processing.
   mobs 0-7 are in the first byte of array
   mobs 8-14 are in the second byte of array
*/
volatile uint8_t g_txd_msg_mobs[2];

// define a fifo for received messages
// messages are added in the ISR, read on demand
struct fifo_can {
	volatile uint8_t w;
	volatile uint8_t r;
	volatile uint8_t count;
	can_msg_t buf[AT90CAN_RECVBUFLEN];
};
static struct fifo_can s_fifo_can_recvd;

// define a fifo for errors
// errors are added in the ISR, read during the handle_interrupt fn
struct fifo_error {
	volatile uint8_t w;
	volatile uint8_t r;
	volatile uint8_t count;
	can_error_t buf[AT90CAN_ERRBUFLEN];
};
static struct fifo_error s_fifo_err;

/*
 * Is a MOB interrupted?
 */
static int at90can_mob_interrupted(int idx)
{
	if (idx < 8)
		return bit_is_set(CANSIT2, idx);
	else
		return bit_is_set(CANSIT1, idx - 8);
}

static void set_mob_flag(volatile uint8_t* flag, int idx) 
{
	if (idx < 8)
		*flag |= _BV(idx);
	else
		*(flag+1) |= _BV(idx - 8);
}

static void clear_mob_flag(volatile uint8_t* flag, int idx) 
{
	if (idx < 8)
		*flag &= ~(_BV(idx));
	else
		*(flag+1) &= ~(_BV(idx - 8));
}

static void new_error(uint8_t err_code, uint8_t dev_buf, uint8_t dev_code)
{
	if (s_fifo_err.count == AT90CAN_ERRBUFLEN)
		return;
	
	// get the address of the next avail buf
	can_error_t* p_err = &s_fifo_err.buf[s_fifo_err.w++];
	s_fifo_err.count++;
	if (s_fifo_err.w == AT90CAN_ERRBUFLEN)
		s_fifo_err.w = 0;
	p_err->error_code = err_code;
	p_err->dev_buffer = dev_buf;
	p_err->dev_code = 0;
}

/*
 * the main CAN controller interrupt routine
 */
ISR(CANIT_vect)
{
	// save the CANPAGE register for restoration at end of ISR
	uint8_t save = CANPAGE;

	// resolve all the MOB interrupts
	for (int i=0; i<15; ++i) {
		if (at90can_mob_interrupted(i)) {
			CANPAGE = i << 4;
			// get the mob status..
			uint8_t status = CANSTMOB;
			// first check for errors
			if (status & MOB_ERROR_MASK) {
				if (status & _BV(BERR)) {
					// set the bit error flag on the mob
					s_at90can_dev_priv.flag |= _BV(5);
					new_error(CAN_MSG_BIT_ERR, i, 0);
				}
				if (status & _BV(SERR)) {
					// set the stuff error flag on the mob
					s_at90can_dev_priv.flag |= _BV(4);
					new_error(CAN_MSG_STUFF_ERR, i, 0);
				}
				if (status & _BV(CERR)) {
					// set the crc error flag on the mob
					s_at90can_dev_priv.flag |= _BV(3);
					new_error(CAN_CRC_ERR, i, 0);
				}
				if (status & _BV(FERR)) {
					// set the form error flag on the mob
					s_at90can_dev_priv.flag |= _BV(2);
					new_error(CAN_FRAME_ERR, i, 0);
				}
				if (status & _BV(AERR)) {
					// set the ack error flag on the mob
					s_at90can_dev_priv.flag |= _BV(1);
					new_error(CAN_ACK_ERR, i, 0);
				}
			} /* end of error checking */
			if (status & _BV(TXOK)) {
				// set the tx flag on the mob
				set_mob_flag(g_txd_msg_mobs, i);
				s_at90can_dev_priv.flag |= _BV(6);
			}
			CANSTMOB = 0;
			// have we received a message...
			if (status & _BV(RXOK)) {

				if (s_fifo_can_recvd.count == AT90CAN_RECVBUFLEN)
					goto end_of_mob;
				
				// copy the message to the fifo...
				// get the address of the next available buf in the fifo
				can_msg_t* p_message = &s_fifo_can_recvd.buf[s_fifo_can_recvd.w++];
				s_fifo_can_recvd.count++;
				if (s_fifo_can_recvd.w == AT90CAN_RECVBUFLEN)
					s_fifo_can_recvd.w = 0;
				
				// Standard ID
				p_message->id =  (uint16_t) CANIDT1 << 3;
				p_message->id |= (uint16_t) CANIDT2 >> 5;
   
				// extended ID
   
				// length
				uint8_t length = (uint8_t)(CANCDMOB & 0x0f);
				p_message->length = length;
   
				// Read Data
				for (uint8_t j=0; j<length; ++j)
					p_message->data[j] = CANMSG;
   
				if (bit_is_set(CANIDT4, RTRTAG))
					p_message->rtr = 1;
				else
					p_message->rtr = 0;

				/* if filters are on, then reset the id so filter is good */
				if (s_at90can_dev_priv.settings->filters.filtering_on) {
					CANIDT1 = (uint8_t)(s_at90can_dev_priv.settings
										->filters.standard_id_filter[i] >> 3);
					CANIDT2 = (uint8_t)(s_at90can_dev_priv.settings
										->filters.standard_id_filter[i] << 5);
					CANIDT3 = 0;
					CANIDT4 = 0;
				}

				s_at90can_dev_priv.flag |= _BV(7);

				/* enable for reception again */
				CANCDMOB = _BV(CONMOB1);
			} else
				CANCDMOB = 0;
		}
	}
end_of_mob:
	// check for error passive mode, this doesn't cause an interrupt
	if (at90can_dev.bus_state == ERROR_ACTIVE && bit_is_set(CANGSTA, ERRP)) {
		at90can_dev.bus_state = ERROR_PASSIVE;
		new_error(CAN_BUS_PASSIVE, 0, 0);
	}
	
	// resolve CANGIT interrupts
	if (CANGIT & GENERAL_ERROR_MASK) {
		if (CANGIT & _BV(BOFFIT)) {
			CANGIT |= _BV(BOFFIT);
			// code for busoff
			s_at90can_dev_priv.flag |= _BV(0);
			at90can_dev.bus_state = BUS_OFF;
			new_error(CAN_BUS_OFF, 0, 0);
		}
		if (CANGIT & _BV(SERG)) {
			CANGIT |= _BV(SERG);
			// code for stuff error general
			s_at90can_dev_priv.flag |= _BV(4);
			new_error(CAN_MSG_STUFF_ERR, 0, 0);
		}
		if (CANGIT & _BV(CERG)) {
			CANGIT |= _BV(CERG);
			// code for crc error general
			s_at90can_dev_priv.flag |= _BV(3);
			new_error(CAN_CRC_ERR, 0, 0);
		}
		if (CANGIT & _BV(FERG)) {
			CANGIT |= _BV(FERG);
			// code for form error general
			s_at90can_dev_priv.flag |= _BV(2);
			new_error(CAN_FRAME_ERR, 0, 0);
		}
		if (CANGIT & _BV(AERG)) {
			CANGIT |= _BV(AERG);
			// code for acknowledgment error general
			s_at90can_dev_priv.flag |= _BV(1);
			new_error(CAN_ACK_ERR, 0, 0);
		}
	}
	
	CANPAGE = save;

}

/*
 * Is a MOB buffer busy?
 */
static int at90can_mob_buffer_enabled(int idx)
{
	if (idx < 8)
		return bit_is_set(CANEN2, idx);
	else
		return bit_is_set(CANEN1, idx - 8);
}

static void at90can_install_filters(void)
{
	can_init_t *settings = s_at90can_dev_priv.settings;

    /*
     *  install the Filters
     */

#ifdef AT90CANDEBUG
	DEVICE_PRINT2("filters:%x", settings->filters.filtering_on);
#endif
	// no filters
	if (settings->filters.filtering_on == 0)
	{
		// here we set the first 8 mobs for reception, the next 7
		// are for transmission
		for (int i=0; i<8; ++i) {
			CANPAGE = i << 4;
			// set the can id mask registers to accept everything
			CANIDM1 = 0;
			CANIDM2 = 0;
			CANIDM3 = 0;
			CANIDM4 = 0;
			// enable for reception
			CANCDMOB = _BV(CONMOB1);
		}
				
        s_at90can_dev_priv.first_tx_mob = 8;
	}
	// yes filters
	else
	{
#ifdef AT90CANDEBUG
		DEVICE_PRINT2("Number of filters:%x", settings->filters.num_filters);
#endif   
		// setup the masks, filters
		for (int i=0; i<settings->filters.num_filters; ++i)
		{
			CANPAGE = i << 4;
			// set the can id mask registers
			CANIDM1 = (uint8_t)(settings->filters.standard_id_mask[i] >> 3);
			CANIDM2 = (uint8_t)(settings->filters.standard_id_mask[i] << 5);
			CANIDM3 = 0;
			CANIDM4 = _BV(RTRMSK)|_BV(IDEMSK);
#ifdef AT90CANDEBUG
			DEVICE_PRINT2("CANIDM%d:%x %x %x %x", i, CANIDM1, CANIDM2,
						  CANIDM3, CANIDM4);
#endif   
			// set filter registers
			CANIDT1 = (uint8_t)(settings->filters.standard_id_filter[i] >> 3);
			CANIDT2 = (uint8_t)(settings->filters.standard_id_filter[i] << 5);
			CANIDT3 = 0;
			CANIDT4 = 0;
#ifdef AT90CANDEBUG
			DEVICE_PRINT2("CANIDT%d:%x %x %x %x", i, CANIDT1, CANIDT2,
						  CANIDT3, CANIDT4);
#endif   
			// enable for reception
			CANCDMOB = _BV(CONMOB1);
		}
        s_at90can_dev_priv.first_tx_mob = settings->filters.num_filters;
        if (s_at90can_dev_priv.first_tx_mob >= 15) {
#ifdef AT90CANDEBUG
            DEVICE_PRINT("Number of filters doesn't allow any transmission MOBS");
#endif
        }
	} /* end setting filters */
}

/*
 * Initialize the AT90CAN controller
 * returns: CAN_OK, or CAN_FAILINIT
 */
uint8_t at90can_init(can_init_t* settings)
{
	// because we may be called at any time, disable the interrupts, put
	// device in listen mode
	CANGIE &= ~_BV(ENIT);
	CANGCON &= _BV(ENASTB);
	// loop until we are disabled
	while(bit_is_set(CANGSTA, ENFG));
	
	uint8_t cfg1 = 0, cfg2 = 0, cfg3 = 0;

	/* set the global variable */
	s_at90can_dev_priv.settings = settings;

	/* clear the flags, this needs to be done even though
	   zero'd on reset, because we may call init at any time
	   not just after reset */
	for (int i=0; i<2; ++i)
		g_txd_msg_mobs[i] = 0;

	switch (settings->speed_setting) {
	case (CAN_20KBPS):
#ifdef AT90CANDEBUG
		DEVICE_PRINT("20kbps");
		DEVICE_PRINT("no timing settings for this speed");
#endif
		return CAN_FAILINIT;
	case (CAN_100KBPS):
#ifdef AT90CANDEBUG
		DEVICE_PRINT("100kbps");
#endif
#ifdef AT90CAN_8MHZ
		cfg1 = AT90CAN_8MHZ_100kBPS_CFG1;
		cfg2 = AT90CAN_8MHZ_100kBPS_CFG2;
		cfg3 = AT90CAN_8MHZ_100kBPS_CFG3;
#endif
#ifdef AT90CAN_16MHZ
		cfg1 = AT90CAN_16MHZ_100kBPS_CFG1;
		cfg2 = AT90CAN_16MHZ_100kBPS_CFG2;
		cfg3 = AT90CAN_16MHZ_100kBPS_CFG3;
#endif
		break;
	case (CAN_125KBPS):
#ifdef AT90CANDEBUG
		DEVICE_PRINT("125kbps");
#endif
#ifdef AT90CAN_8MHZ
		cfg1 = AT90CAN_8MHZ_125kBPS_CFG1;
		cfg2 = AT90CAN_8MHZ_125kBPS_CFG2;
		cfg3 = AT90CAN_8MHZ_125kBPS_CFG3;
#endif
#ifdef AT90CAN_16MHZ
		cfg1 = AT90CAN_16MHZ_125kBPS_CFG1;
		cfg2 = AT90CAN_16MHZ_125kBPS_CFG2;
		cfg3 = AT90CAN_16MHZ_125kBPS_CFG3;
#endif
		break;
	case (CAN_250KBPS):
#ifdef AT90CANDEBUG
		DEVICE_PRINT("250kbps");
#endif
#ifdef AT90CAN_8MHZ
		cfg1 = AT90CAN_8MHZ_250kBPS_CFG1;
		cfg2 = AT90CAN_8MHZ_250kBPS_CFG2;
		cfg3 = AT90CAN_8MHZ_250kBPS_CFG3;
#endif
#ifdef AT90CAN_16MHZ
		cfg1 = AT90CAN_16MHZ_250kBPS_CFG1;
		cfg2 = AT90CAN_16MHZ_250kBPS_CFG2;
		cfg3 = AT90CAN_16MHZ_250kBPS_CFG3;
#endif
		break;
	case (CAN_500KBPS):
#ifdef AT90CANDEBUG
		DEVICE_PRINT("500kbps");
#endif
#ifdef AT90CAN_8MHZ
		cfg1 = AT90CAN_8MHZ_500kBPS_CFG1;
		cfg2 = AT90CAN_8MHZ_500kBPS_CFG2;
		cfg3 = AT90CAN_8MHZ_500kBPS_CFG3;
#endif
#ifdef AT90CAN_16MHZ
		cfg1 = AT90CAN_16MHZ_500kBPS_CFG1;
		cfg2 = AT90CAN_16MHZ_500kBPS_CFG2;
		cfg3 = AT90CAN_16MHZ_500kBPS_CFG3;
#endif
		break;
	case (CAN_1000KBPS):
#ifdef AT90CANDEBUG
		DEVICE_PRINT("1000kbps");
#endif
#ifdef AT90CAN_8MHZ
		// we are not allowed to use this setting
		return CAN_FAILINIT;
#endif
#ifdef AT90CAN_16MHZ
		cfg1 = AT90CAN_16MHZ_1000kBPS_CFG1;
		cfg2 = AT90CAN_16MHZ_1000kBPS_CFG2;
		cfg3 = AT90CAN_16MHZ_1000kBPS_CFG3;
#endif
		break;
	default:
#ifdef AT90CANDEBUG
		DEVICE_PRINT("illegal bit rate");
#endif
		return CAN_FAILINIT;
	}

	// set the baud rate
	CANBT1 = cfg1;
	CANBT2 = cfg2;
	CANBT3 = cfg3;

	// clear each MOB status and control register
	for (int i=0; i<15; ++i) {
		CANPAGE = i << 4;
		CANCDMOB = 0;
		CANSTMOB = 0;
	}

	// enable interrupts
	CANGIE |= _BV(ENBOFF)|_BV(ENRX)|_BV(ENTX)|_BV(ENERR)|_BV(ENERG);

	// enable interrupts on all mobs
	CANIE1 = 0x7f;
	CANIE2 = 0xff;

	at90can_install_filters();

	// put the controller in listening mode, if loopback is requested
	if (settings->loopback_on) {
#ifdef AT90CANDEBUG
		DEVICE_PRINT2("setting loopback mode:%x", settings->loopback_on);
#endif   
		CANGCON |= _BV(LISTEN);
	}

	// turn on the controller
	CANGCON |= _BV(ENASTB);
	// loop until we are enabled
	while(!bit_is_set(CANGSTA, ENFG));
	
    // Activate the Interrupts
	CANGIE |= _BV(ENIT);

#ifdef AT90CANDEBUG
    DEVICE_PRINT("controller enabled");
#endif   
	return CAN_OK;
}

/*
 * returns: CAN_OK, CAN_FAIL
 */
uint8_t at90can_self_test(void)
{
	// we are enabled
	if (!bit_is_set(CANGSTA, ENFG))
		return CAN_FAIL;
	// interrupts enabled
	if (!bit_is_set(CANGIE, ENIT))
		return CAN_FAIL;
	// not in bus-off mode
	if (bit_is_set(CANGSTA, BOFF))
		return CAN_FAIL;
	// not in error passive mode
	if (bit_is_set(CANGSTA, ERRP))
		return CAN_FAIL;
	// if in loopback mode, then is listen mode set
	if (s_at90can_dev_priv.settings->loopback_on && !bit_is_set(CANGCON, LISTEN))
		return CAN_FAIL;
	return CAN_OK;
}

/* search for a mob that is not enabled, so that
   we can transmit a message. the index of the
   free mob will be put in the argument. Returns
   CAN_ALL_TXBUSY if no mob available, and
   CAN_OK if one was found */
uint8_t at90can_get_next_free_tx_mob(int* txbuf_n)
{
	int i;
	*txbuf_n = -1;
	for (i=s_at90can_dev_priv.first_tx_mob; i<15; ++i) {
		if (at90can_mob_buffer_enabled(i) == 0) {
			CANPAGE = i << 4;
			if (CANSTMOB == 0) {
				*txbuf_n = i;
				break;
			}
		}
	}

	if (i == 15)
		/* all buffers busy, can't send */
		return CAN_ALLTXBUSY;

#ifdef AT90CANDEBUG
	DEVICE_PRINT2("next tx mob:%x", *txbuf_n);
#endif	
	return CAN_OK;
}

/*
 * at90can_write_msg
 *
 */
void at90can_write_msg(int tx_mob_idx, const can_msg_t *p_message)
{
#ifdef AT90CANDEBUG
	DEVICE_PRINT2("write_msg:%x", tx_mob_idx);
#endif	

	// select the mob, plus autoinc data buffer, with start of zero
	CANPAGE = tx_mob_idx << 4;

	// Standard ID
	CANIDT1 = (uint8_t) (p_message->id>>3);
	CANIDT2 = (uint8_t) (p_message->id<<5);
   
	// Extended ID
	CANIDT3 = 0x00;

	// is this "Remote Transmit Request" ?
	if (p_message->rtr)
		CANIDT4 = _BV(RTRTAG);
	else
		CANIDT4 = 0x00;
   
	uint8_t length = p_message->length;
   
	if (length > 8)
		length = 8;

	// Data
	for (uint8_t i=0;i<length;i++)
		CANMSG = p_message->data[i];

	// now send it...
	CANCDMOB = _BV(CONMOB0) + length;
}

/*
 * check for a CAN message
 * returns: CAN_MSGAVAIL or CAN_NOMSG
 */
uint8_t at90can_check_msg_received(void)
{
	uint8_t recv_buf_count;
	ATOMIC_BLOCK(ATOMIC_FORCEON)
	{
		recv_buf_count = s_fifo_can_recvd.count;
	}
	if (recv_buf_count)
		return CAN_MSGAVAIL;
	return CAN_NOMSG;
}

/*
 * read a CAN message
 */
uint8_t at90can_read_msg(can_msg_t *p_message)
{
	uint8_t recv_buf_count;
	ATOMIC_BLOCK(ATOMIC_FORCEON)
	{
		recv_buf_count = s_fifo_can_recvd.count;
	}
	if (recv_buf_count == 0)
		return CAN_NOMSG;

#ifdef MCPDEBUG
	DEVICE_PRINT("read_msg");
#endif

	ATOMIC_BLOCK(ATOMIC_FORCEON)
	{
		uint8_t i = s_fifo_can_recvd.r;
		memcpy(p_message, &s_fifo_can_recvd.buf[i], sizeof(can_msg_t));
		s_fifo_can_recvd.count--;
		if (++i >= AT90CAN_RECVBUFLEN)
			i = 0;
		s_fifo_can_recvd.r = i;
	}
	return CAN_OK;

}

/*
 * handle interrupts, clearing flags
 * the current status flag is put into the arg status_flag
 * returns CAN_INTERRUPT if interrupt has occurred
 * CAN_NOINTERRUPT if no interrupt
 */
uint8_t at90can_handle_interrupt(uint8_t* status_flag)
{
	ATOMIC_BLOCK(ATOMIC_FORCEON) 
	{
		*status_flag = s_at90can_dev_priv.flag;
		s_at90can_dev_priv.flag = 0;
	}
	if (*status_flag) {
		
		// if we have transmit complete flags, clear them
		ATOMIC_BLOCK(ATOMIC_FORCEON) 
		{
			if (g_txd_msg_mobs[0] || g_txd_msg_mobs[1])
				g_txd_msg_mobs[0] = g_txd_msg_mobs[1] = 0;
		}
	
		// now do the errors...
		uint8_t count = 0;
		ATOMIC_BLOCK(ATOMIC_FORCEON)
		{
			count = s_fifo_err.count;
		}
		while (count > 0) {
			uint8_t i=0;
			ATOMIC_BLOCK(ATOMIC_FORCEON)
			{
				i = s_fifo_err.r;
				count = --s_fifo_err.count;
				if (++s_fifo_err.r >= AT90CAN_ERRBUFLEN)
					s_fifo_err.r = 0;
			}
			can_handle_error(&s_fifo_err.buf[i]);
		}
	}

	return *status_flag ? CAN_INTERRUPT : CAN_NOINTERRUPT;
}

// called to get rx,tx error counts
// returns CAN_OK, CAN_FAIL
uint8_t at90can_error_counts(uint8_t* tx_count, uint8_t* rx_count)
{
	ATOMIC_BLOCK(ATOMIC_FORCEON)
	{
		*tx_count = CANTEC;
		*rx_count = CANREC;
	}
	return CAN_OK;
}

void at90can_clear_tx_buffers(void)
{
	// abort pending transmissions
	ATOMIC_BLOCK(ATOMIC_FORCEON)
	{
		CANGCON |= _BV(ABRQ);
	}
	uint8_t en1, en2;
	// loop until all MOBS disabled
	do {
		ATOMIC_BLOCK(ATOMIC_FORCEON)
		{
			en1 = CANEN1;
			en2 = CANEN2;
		}
	} while (en1 || en2);
	// now clear the ABORT flag
	ATOMIC_BLOCK(ATOMIC_FORCEON)
	{
		CANGCON &= ~_BV(ABRQ);
	}
	// BUGBUG: what do we do about stuff being received while we are
	// doing this, or above when aborting?
	at90can_install_filters();
}



