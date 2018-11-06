#include <avr/io.h>
#include <util/delay.h>
#include <util/atomic.h>
#include <string.h>
#include "mcp2515.h"
#include "mcp2515_def.h"
#include "spi.h"
#include "globals.h"
#include "driver.h"

// define the following to debug SPI comm's
// note that defining this causes hangs as
// it tries to write to serial port and
// interrupts are disabled sometimes, so
// this is not much use
// ----------------------------------------
/* #define MCPSPIDEBUG 1 */

// globals

static const char* k_name = "mcp2515:";

// device specific struct
static struct mcp2515_dev_priv* s_mcp2515_dev_p;

// define a fifo for received messages
// messages are added in the ISR, read on demand
struct fifo_can {
	volatile uint8_t w;
	volatile uint8_t r;
	volatile uint8_t count;
	can_msg_t buf[MCP_RECVBUFLEN];
};
static struct fifo_can s_fifo_can_recvd;

// define a fifo for errors
// errors are added in the ISR, read during the handle_interrupt fn
struct fifo_error {
	volatile uint8_t w;
	volatile uint8_t r;
	volatile uint8_t count;
	can_error_t buf[MCP_ERRBUFLEN];
};
static struct fifo_error s_fifo_err;

// set CS low to select device
static void mcp2515_select(void)
{
    PORT_CANCS &= ~(_BV(P_CANCS));
}

// set CS high to unselect device
static void mcp2515_unselect(void)
{
    PORT_CANCS |= _BV(P_CANCS);
}

/*
 * read the rx,tx status, s/b called in ATOMIC_BLOCK
 */
static int mcp2515_read_status(void)
{
	// Get MCP2515 status
	mcp2515_select();
	spi_transfer(MCP_SPI_READ_STATUS);
	uint8_t status = spi_transfer(0xff);
	// repeat the status 2nd byte
	spi_transfer(0xff);
	mcp2515_unselect();
	return status;
}

/*
 * write a register on the MCP2515
 */
static void mcp2515_write_register( uint8_t address, uint8_t data )
{
#ifdef MCPSPIDEBUG
	DEVICE_PRINT2("write_register:%x,%x", address, data);
#endif

	mcp2515_select();
    spi_transfer(MCP_SPI_WRITE);
    spi_transfer(address);
    spi_transfer(data);
	mcp2515_unselect();
	
#ifdef MCPSPIDEBUG
	DEVICE_PRINT("done write_register");
#endif
}

/*
 * read a register on the MCP2515
 */
static uint8_t mcp2515_read_register(uint8_t address)
{
#ifdef MCPSPIDEBUG
	DEVICE_PRINT2("read_register:%x", address);
#endif
	mcp2515_select();
    spi_transfer(MCP_SPI_READ);
    spi_transfer(address);
    uint8_t data = spi_transfer(0xff);
	mcp2515_unselect();
	
#ifdef MCPSPIDEBUG
	DEVICE_PRINT2("done read_register:%x", data);
#endif   
    return data;
}

/*
 * modify a bit on the MCP2515
 */
static void mcp2515_bit_modify(uint8_t address, uint8_t mask, uint8_t data)
{
#ifdef MCPSPIDEBUG
	DEVICE_PRINT2("bit_modify:%x,%x,%x", address, mask, data);
#endif

	mcp2515_select();
    spi_transfer(MCP_SPI_BIT_MOD);
    spi_transfer(address);
    spi_transfer(mask);
    spi_transfer(data);
	mcp2515_unselect();

#ifdef MCPSPIDEBUG
	DEVICE_PRINT("done bit_modify");
#endif   
}

/*
 * Check for received CAN messages
 */
int mcp2515_check_receive(struct can_device* dev)
{
#ifdef MCPDEBUG
	DEVICE_PRINT("check_receive");
#endif
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
 * Initialize the MCP2515
 */
int mcp2515_init(can_init_t* settings, struct can_device* dev)
{
    /* set the device in the priv structure */
    s_mcp2515_dev_p = dev->priv_dev;
    dev->priv_dev->init = 0;
    
    /* initialize the device structure, except for priv_dev, which should be set already */
    dev->bus_state = ERROR_ACTIVE;
    dev->init_fn = mcp2515_init;
    dev->reinit_fn = mcp2515_reinit;
    dev->self_test_fn = mcp2515_self_test;
    dev->check_receive_fn = mcp2515_check_receive;
    dev->free_send_buffer_fn = mcp2515_get_next_free_tx_buf;
    dev->write_msg_fn = mcp2515_write_msg;
    dev->read_msg_fn = mcp2515_read_msg;
    dev->device_command = 0;
    dev->handle_int_fn = mcp2515_handle_interrupt;
    dev->error_counts = mcp2515_error_counts;
    dev->clear_tx_buffers = mcp2515_clear_tx_buffers;
    
    dev->settings = settings;

    return mcp2515_reinit(dev);
}

/*
 * Initialize the MCP2515
 */
int mcp2515_reinit(struct can_device* dev)
{
	uint8_t cfg1, cfg2, cfg3;

    can_init_t* settings = dev->settings;
    if (settings == 0)
        return CAN_FAILINIT;

    // if device has been running, clear tx buffers
    if (dev->priv_dev->init)
        mcp2515_clear_tx_buffers(dev);
    
#ifdef MCPDEBUG
	DEVICE_PRINT2("setting loopback mode:%x", settings->loopback_on);
#endif   

	switch (settings->speed_setting) {
	case (CAN_20KBPS):
#ifdef MCPDEBUG
		DEVICE_PRINT("20kbps");
#endif
#ifdef MCP2515_8MHZ
		cfg1 = MCP_8MHZ_20kBPS_CFG1;
		cfg2 = MCP_8MHZ_20kBPS_CFG2;
		cfg3 = MCP_8MHZ_20kBPS_CFG3;
#endif
#ifdef MCP2515_16MHZ
		cfg1 = MCP_16MHZ_20kBPS_CFG1;
		cfg2 = MCP_16MHZ_20kBPS_CFG2;
		cfg3 = MCP_16MHZ_20kBPS_CFG3;
#endif
		break;
	case (CAN_125KBPS):
#ifdef MCPDEBUG
		DEVICE_PRINT("125kbps");
#endif
#ifdef MCP2515_8MHZ
		cfg1 = MCP_8MHZ_125kBPS_CFG1;
		cfg2 = MCP_8MHZ_125kBPS_CFG2;
		cfg3 = MCP_8MHZ_125kBPS_CFG3;
#endif
#ifdef MCP2515_16MHZ
		cfg1 = MCP_16MHZ_125kBPS_CFG1;
		cfg2 = MCP_16MHZ_125kBPS_CFG2;
		cfg3 = MCP_16MHZ_125kBPS_CFG3;
#endif
		break;
	case (CAN_250KBPS):
#ifdef MCPDEBUG
		DEVICE_PRINT("250kbps");
#endif
#ifdef MCP2515_8MHZ
		cfg1 = MCP_8MHZ_250kBPS_CFG1;
		cfg2 = MCP_8MHZ_250kBPS_CFG2;
		cfg3 = MCP_8MHZ_250kBPS_CFG3;
#endif
#ifdef MCP2515_16MHZ
		cfg1 = MCP_16MHZ_250kBPS_CFG1;
		cfg2 = MCP_16MHZ_250kBPS_CFG2;
		cfg3 = MCP_16MHZ_250kBPS_CFG3;
#endif
		break;
	case (CAN_500KBPS):
#ifdef MCPDEBUG
		DEVICE_PRINT("500kbps");
#endif
#ifdef MCP2515_8MHZ
		cfg1 = MCP_8MHZ_500kBPS_CFG1;
		cfg2 = MCP_8MHZ_500kBPS_CFG2;
		cfg3 = MCP_8MHZ_500kBPS_CFG3;
#endif
#ifdef MCP2515_16MHZ
		cfg1 = MCP_16MHZ_500kBPS_CFG1;
		cfg2 = MCP_16MHZ_500kBPS_CFG2;
		cfg3 = MCP_16MHZ_500kBPS_CFG3;
#endif
		break;
	case (CAN_1000KBPS):
#ifdef MCPDEBUG
		DEVICE_PRINT("1000kbps");
#endif
#ifdef MCP2515_8MHZ
		// we are not allowed to use this setting
		return CAN_FAILINIT;
#endif
#ifdef MCP2515_16MHZ
		cfg1 = MCP_16MHZ_1000kBPS_CFG1;
		cfg2 = MCP_16MHZ_1000kBPS_CFG2;
		cfg3 = MCP_16MHZ_1000kBPS_CFG3;
#endif
		break;
	default:
#ifdef MCPDEBUG
		DEVICE_PRINT("illegal bit rate");
#endif
		return CAN_FAILINIT;
	}

    // MCP2515 software reset
    // this puts device in configuration mode
    // this is only mode that allows CNF1,2,3
    // TXRTSCTRL, filter and mask registers to be modified
#ifdef MCPSPIDEBUG
	DEVICE_PRINT("Start reset");
#endif   
	mcp2515_select();
    spi_transfer( MCP_SPI_RESET );
	mcp2515_unselect();
#ifdef MCPSPIDEBUG
	DEVICE_PRINT("done reset");
#endif   

    // delay for a while to let chip settle down
    _delay_ms(20);

	// make sure we are in configuration mode
	if (mcp2515_read_register( MCP_CANSTAT ) != _BV(MCP_OPMOD2)) {
#ifdef MCPSPIDEBUG
		DEVICE_PRINT("not in config mode");
#endif   
		return CAN_FAILINIT;
	}
	
    // BRP
    mcp2515_write_register( MCP_CNF1, cfg1 );
   
    // Prop Seg and Phase Seg1 config
    mcp2515_write_register( MCP_CNF2, cfg2 );
   
    // Wake-up Filter, Phase Seg2 config
    mcp2515_write_register( MCP_CNF3, cfg3 );
   
    // Activate the Error, Tx,Rx Buffer Interrupts
    mcp2515_write_register(MCP_CANINTE,
			   _BV(MCP_ERRIE)|_BV(MCP_RX1IE)|_BV(MCP_RX0IE)|_BV(MCP_TX0IE)|_BV(MCP_TX1IE)|_BV(MCP_TX2IE));

    /*
     *  install the Filters
     */

#ifdef MCPDEBUG
	DEVICE_PRINT2("filters:%x", settings->filters.filtering_on);
#endif
	// no filters
	if (settings->filters.filtering_on == 0)
	{
		// Buffer 0 : receive all messages
		mcp2515_write_register( MCP_RXB0CTRL, _BV(MCP_RXM1) | _BV(MCP_RXM0) );
   
		// Buffer 1 : receive all messages
		mcp2515_write_register( MCP_RXB1CTRL, _BV(MCP_RXM1) | _BV(MCP_RXM0) );
	}
	// yes filters
	else
	{
#ifdef MCPDEBUG
		DEVICE_PRINT2("Number of filters:%x", settings->filters.num_filters);
#endif   
		// Buffer 0 : filter standard id messages, roll over to buffer 1
		mcp2515_write_register( MCP_RXB0CTRL, _BV(MCP_RXM0) | _BV(MCP_BUKT) );

		// Buffer 1 : filter standard id messages
		mcp2515_write_register( MCP_RXB1CTRL, _BV(MCP_RXM0) );

		// setup the masks
		for (int i=0; i<2; ++i)
		{
			// set filter mask
			uint8_t hi = settings->filters.standard_id_mask[i] >> 3;
			uint8_t lo = (settings->filters.standard_id_mask[i] & 0x7) << 5;
			
			uint8_t ehi = settings->filters.extended_id_mask[i] >> 8;
			uint8_t elo = settings->filters.extended_id_mask[i] & 0xff;

			uint8_t reg = MCP_RXM0SIDH + 4 * i;

			// set the filter mask in receive buffer 0
			mcp2515_write_register( reg, hi );
			mcp2515_write_register( reg+1, lo );
			mcp2515_write_register( reg+2, ehi );
			mcp2515_write_register( reg+3, elo );
		}

		// setup the filters
		for (int i=0; i<settings->filters.num_filters; ++i)
		{
			uint8_t hi = settings->filters.standard_id_filter[i] >> 3;
			uint8_t lo = (settings->filters.standard_id_filter[i] & 0x7) << 5;
			
			uint8_t ehi = settings->filters.extended_id_filter[i] >> 8;
			uint8_t elo = settings->filters.extended_id_filter[i] & 0xff;

			uint8_t reg = MCP_RXF0SIDH + 4 * i;

			// gap between 2 and third filter addresses
			if (i == 3)
				reg = MCP_RXF3SIDH;
			
			// set the filter mask in receive buffer 0
			mcp2515_write_register( reg, hi );
			mcp2515_write_register( reg + 1, lo );
			mcp2515_write_register( reg + 2, ehi );
			mcp2515_write_register( reg + 3, elo );
		}

	} /* end setting filters */
	
    // Set all receive data register header bits to 0
    mcp2515_write_register( MCP_RXB0SIDH, 0 );
    mcp2515_write_register( MCP_RXB0SIDL, 0 );
    mcp2515_write_register( MCP_RXB0EID8, 0 );
    mcp2515_write_register( MCP_RXB0EID0, 0 );
   
    mcp2515_write_register( MCP_RXB1SIDH, 0 );
    mcp2515_write_register( MCP_RXB1SIDL, 0 );
    mcp2515_write_register( MCP_RXB1EID8, 0 );
    mcp2515_write_register( MCP_RXB1EID0, 0 );
   
    // deactivate the RXnBF Pins (High Impedance State)
    mcp2515_write_register( MCP_BFPCTRL, 0 );
   
    // TXnRTS pins deactivated
    mcp2515_write_register( MCP_TXRTSCTRL, 0 );
   
    // Device to normal mode (or loopback)
	mcp2515_write_register( MCP_CANCTRL, (settings->loopback_on ? _BV(MCP_REQOP1) : 0) );

	// delay a long time to allow clock to stabilize
	_delay_ms(500);
	
	// CAN_INT, input, also activate the pullup resistor
	*dev->dev_priv->ddr_port &= ~dev->dev_priv->port_pin;
	*dev->dev_priv->port |= dev->dev_priv->port_pin;

    // setup interrupt for CAN
	*dev->dev_priv->int_dir_reg |= dev->dev_priv->int_dir_mask;
	*dev->dev_priv->int_en_reg |= dev->dev_priv->int_en_mask;

    // set the initialized flag
    dev->dev_priv->init = 1;
    
	return CAN_OK;
}

/*
 * Returns: CAN_OK, CAN_FAIL
 */
int mcp2515_self_test(struct can_device* dev)
{
	// read the CANCTRL register, ensure that it matches setup
	uint8_t reg = mcp2515_read_register( MCP_CANCTRL );
	if (reg != (dev->settings->loopback_on ? _BV(MCP_REQOP1) : 0))
		return CAN_FAIL;
	// read the CANSTAT register, ensure in normal mode
	reg = mcp2515_read_register( MCP_CANSTAT );
	// mask off bits other than mode (interrupts)
	// should be in normal operation mode
	uint8_t tstval = dev->settings->loopback_on ? _BV(MCP_OPMOD1) : 0;
	if ((reg & (_BV(MCP_OPMOD0)|_BV(MCP_OPMOD1)|_BV(MCP_OPMOD2))) != tstval)
		return CAN_FAIL;
	return CAN_OK;
}

int mcp2515_get_next_free_tx_buf(struct can_device* dev, int* txbuf_n)
{
	*txbuf_n = 0;

	ATOMIC_BLOCK(ATOMIC_FORCEON)
	{
		// Get MCP2515 status
		uint8_t status = mcp2515_read_status();
		
		// test the status byte to find a vacant tx buffer
		if (bit_is_clear(status, MCP_TXB0CNTRL_TXREQ))
			*txbuf_n = 0;
		else if (bit_is_clear(status, MCP_TXB1CNTRL_TXREQ))
			*txbuf_n = 1;
		else if (bit_is_clear(status, MCP_TXB2CNTRL_TXREQ))
			*txbuf_n = 2;
		else
			/* all buffers busy, can't send */
			return CAN_ALLTXBUSY;
	}
#ifdef MCPDEBUG
	DEVICE_PRINT2("next tx buf:%x", *txbuf_n);
#endif	
	return CAN_OK;
}

/*
 * mcp2515_write_msg
 *
 */
void mcp2515_write_msg(struct can_device* dev, int txbuf_n,
                       const can_msg_t *p_message)
{
#ifdef MCPDEBUG
	DEVICE_PRINT2("write_msg:%x", txbuf_n);
#endif

	ATOMIC_BLOCK(ATOMIC_FORCEON)
	{
		mcp2515_select();
		uint8_t cmd = MCP_SPI_LOAD_TX;
		if (txbuf_n == 1)
			cmd |= 0x2;
		else if (txbuf_n == 2)
			cmd |= 0x4;
		spi_transfer(cmd);      /* CTRL */

        if (p_message->idtype == CAN_STANDARD_ID)
        {
            // Standard ID
            spi_transfer((uint8_t) (p_message->id>>3)); /* SIDH */
            spi_transfer((uint8_t) (p_message->id<<5)); /* SIDL */
   
            // Extended ID
            spi_transfer(0x00);     /* EID8 */
            spi_transfer(0x00);     /* EID0 */
        } else {
            // Standard ID
            spi_transfer((uint8_t)(p_message->id >> 3)); /* SIDH */
            spi_transfer((uint8_t)(p_message->id << 5)
                         | 0x8
                         | (uint8_t)(0x3 & (p_message->id >> 27))); /* SIDL + EXIDE + bit 18 and 17 */
   
            // Extended ID
            spi_transfer((uint8_t)(p_message->id >> 19));     /* EID15-EID8 */
            spi_transfer((uint8_t)(p_message->id >> 11));     /* EID8-EID0 */
        }
   
		uint8_t length = p_message->length;
   
		if (length > 8)
			length = 8;
   
		// is this "Remote Transmit Request" ?
		if (p_message->rtr)
			// data len + RTR
			spi_transfer(_BV(MCP_RTR) | length); /* DLC */
		else
			// data len
			spi_transfer(length); /* DLC */
       
		// Data
		for (uint8_t i=0;i<length;i++)
			spi_transfer(p_message->data[i]); /* D1-D8 */

		mcp2515_unselect();
		
		// signal ready-to-send
		cmd = MCP_SPI_RTS;
		if (txbuf_n == 0)
			cmd |= 0x1;
		else if (txbuf_n == 1)
			cmd |= 0x2;
		else
			cmd |= 0x4;

		mcp2515_select();
		spi_transfer(cmd);
		mcp2515_unselect();
	}
}

/*
 * read a CAN message
 */
int mcp2515_read_msg(struct can_device* dev, can_msg_t *p_message)
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
		if (++i >= MCP_RECVBUFLEN)
			i = 0;
		s_fifo_can_recvd.r = i;
	}
	return CAN_OK;
}

/*
 * handle interrupts, clearing TX flags
 * the current status is put into the arg status_flag
 * if the status is not zero, then there is work to
 * be done in polling, ie reading messages
 * rx flags are cleared by can_get_message instead
 */
int mcp2515_handle_interrupt(struct can_device* dev, int* status_flag)
{
	ATOMIC_BLOCK(ATOMIC_FORCEON)
	{
		*status_flag = dev->dev_priv->flag;
		dev->dev_priv->flag = 0;
	}

	// if flag is set, we have an interrupt
	if (*status_flag) {

#ifdef MCPDEBUG
		DEVICE_PRINT("handle_interrupt");
#endif	

		ATOMIC_BLOCK(ATOMIC_FORCEON)
		{
			// read the current status - add to the flag
			// will already contain status of receive flags from
			// interrupt routine
			*status_flag |= mcp2515_read_status();

			uint8_t mask = 0;
			// set the mask for each interrupt flag found
			if (bit_is_set(*status_flag, MCP_CANINTF_TX0IF))
				mask |= _BV(MCP_TX0IF);
			if (bit_is_set(*status_flag, MCP_CANINTF_TX1IF))
				mask |= _BV(MCP_TX1IF);
			if (bit_is_set(*status_flag, MCP_CANINTF_TX2IF))
				mask |= _BV(MCP_TX2IF);
			
			// send the command to clear interrupt flag(s)
			if (mask)
			  mcp2515_bit_modify(MCP_CANINTF, mask, 0);
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
				if (++s_fifo_err.r >= MCP_ERRBUFLEN)
					s_fifo_err.r = 0;
			}
			can_handle_error(dev, &s_fifo_err.buf[i]);
		}

	}

	uint8_t recv_buf_count;
	ATOMIC_BLOCK(ATOMIC_FORCEON)
	{
		recv_buf_count = s_fifo_can_recvd.count;
	}
	
	return (*status_flag || recv_buf_count > 0) ?
		CAN_INTERRUPT : CAN_NOINTERRUPT;
}

int mcp2515_error_counts(struct can_device* dev, uint8_t* tx_count,
                         uint8_t* rx_count)
{
	ATOMIC_BLOCK(ATOMIC_FORCEON)
	{
		*tx_count = mcp2515_read_register( MCP_TEC );
		*rx_count = mcp2515_read_register( MCP_REC);
	}
	return CAN_OK;
}

void mcp2515_clear_tx_buffers(struct can_device* dev)
{
	// abort pending transmissions
	ATOMIC_BLOCK(ATOMIC_FORCEON)
	{
		mcp2515_bit_modify(MCP_CANCTRL, _BV(MCP_ABAT), 1);
	}
	uint8_t status;
	// loop until all TXREQ flags are clear
	do {
		ATOMIC_BLOCK(ATOMIC_FORCEON)
		{
			status = mcp2515_read_status();
		}
	} while ((status & _BV(MCP_TXB0CNTRL_TXREQ))
			 || (status & _BV(MCP_TXB0CNTRL_TXREQ))
			 || (status & _BV(MCP_TXB0CNTRL_TXREQ)));
	// now clear the ABORT flag
	ATOMIC_BLOCK(ATOMIC_FORCEON)
	{
		mcp2515_bit_modify(MCP_CANCTRL, _BV(MCP_ABAT), 0);
	}
}

/*
 * Either a pin change interrupt, or an interrupt triggered by falling edge
 * depending on whether MCP2515_INT_VEC_ANY_CHANGE is defined
 */
ISR(MCP2515_INT_VECT)
{
	can_error_t* p_err;
	
#if defined(MCP2515_INT_VECT_ANY_CHANGE)
	// see if can interrupt pin is lo, ie we got a falling edge
	if (bit_is_clear(*s_mcp2515_dev_priv.pin, s_mcp2515_dev_priv.port_pin))
		s_mcp2515_dev_priv.flag = 1;
	else
		return;
#else
	s_mcp2515_dev_priv.flag = 1;
#endif

	// read interrupt flag register
	uint8_t iflags = mcp2515_read_register(MCP_CANINTF);

	// check for received messages...
	while (bit_is_set(iflags, MCP_RX0IF) || bit_is_set(iflags, MCP_RX1IF)) {

		// make sure we have space in the fifo...
		if (s_fifo_can_recvd.count == MCP_RECVBUFLEN) {
			// record an error
			if (s_fifo_err.count == MCP_ERRBUFLEN)
				goto out_of_here;
			// get the address of the next available buf in the fifo
			p_err = &s_fifo_err.buf[s_fifo_err.w++];
			if (s_fifo_err.w == MCP_ERRBUFLEN)
				s_fifo_err.w = 0;
			s_fifo_err.count++;
			p_err->error_code = CAN_RECV_BUF_OVERFLOW;
			p_err->dev_buffer = 0;
			p_err->dev_code = 0;
			goto after_recv;
		}
		
		// get the address of the next available buf in the fifo
		can_msg_t* p_message = &s_fifo_can_recvd.buf[s_fifo_can_recvd.w++];
		if (s_fifo_can_recvd.w == MCP_RECVBUFLEN)
			s_fifo_can_recvd.w = 0;
		s_fifo_can_recvd.count++;

		// calculate the SPI command, this also clears the received flag for the
		// interrupt when CS is put high again
		uint8_t cmd = bit_is_set(iflags, MCP_RX0IF) ?
			MCP_SPI_READ_RX0 : MCP_SPI_READ_RX1;

		mcp2515_select();

		spi_transfer(cmd);

		// Standard ID
		p_message->id =  (uint16_t) spi_transfer(0xff) << 3; /* SID3-10 */
		uint8_t data = spi_transfer(0xff);
		p_message->id |= (uint16_t)(data >> 5); /* SID0-2, SRR, IDE, EID17-16 */
		p_message->rtr = (data & _BV(4)) ? 1 : 0;
        p_message->idtype = (data & _BV(3)) >> 3;
        if (p_message->idtype == CAN_EXTENDED_ID)
        {
            // extended ID
            spi_transfer(0xff);
            p_message->id |= (uint32_t)(data << 11); /* EID8-EID15 */
            spi_transfer(0xff);
            p_message->id |= (uint32_t)(data << 19); /* EID16-EID17 */
        } else {
            // standard ID
            spi_transfer(0xff);
            spi_transfer(0xff);
        }
           
		// length 
		uint8_t length = spi_transfer(0xff) & 0x0f;
		p_message->length = length;
   
		// Read Data
		for (uint8_t i=0;i<length;i++)
			p_message->data[i] = spi_transfer(0xff);

		// also clears the interrupt flag
		mcp2515_unselect();

		// read interrupt flag register
		iflags = mcp2515_read_register(MCP_CANINTF);

	} /* end of received messages */
after_recv:
	// do we have an error condition?
	if (bit_is_set(iflags, MCP_ERRIF)) {
		// do the error fifo
		if (s_fifo_err.count == MCP_ERRBUFLEN)
			goto out_of_here;
		// get the address of the next available buf in the fifo
		p_err = &s_fifo_err.buf[s_fifo_err.w++];
		if (s_fifo_err.w == MCP_ERRBUFLEN)
			s_fifo_err.w = 0;
		s_fifo_err.count++;

		// read the error register
		uint8_t err = mcp2515_read_register(MCP_EFLG);

		// receive buffer 0 overflow
		if (err & _BV(MCP_RX0OVR)) {
			p_err->error_code = CAN_RECV_BUF_OVERFLOW;
			p_err->dev_buffer = 0;
			p_err->dev_code = 0;
			mcp2515_bit_modify(MCP_EFLG, _BV(MCP_RX0OVR), 0);
			// receive buffer 1 overflow
		} else if (err & _BV(MCP_RX1OVR)) {
			p_err->error_code = CAN_RECV_BUF_OVERFLOW;
			p_err->dev_buffer = 0;
			p_err->dev_code = 0;
			mcp2515_bit_modify(MCP_EFLG, _BV(MCP_RX1OVR), 0);
			// bus off
		} else if (err & _BV(MCP_TXBO)) {
			p_err->error_code = CAN_BUS_OFF;
			p_err->dev_buffer = 0;
			// if already bus off, then now error-active
			if (s_mcp2515_dev_priv.dev->bus_state == BUS_OFF) {
				s_mcp2515_dev_priv.dev->bus_state = ERROR_ACTIVE;
				p_err->dev_code = 0;
			} else {
				// bus off state
				s_mcp2515_dev_priv.dev->bus_state = BUS_OFF;
				p_err->dev_code = 1;
			}
			// error-passive TX or RX
		} else if ((err & _BV(MCP_TXEP)) || (err & _BV(MCP_RXEP))) {
			p_err->error_code = CAN_BUS_PASSIVE;
			p_err->dev_buffer = 0;
			// if already error-passive, then now error-active
			if (s_mcp2515_dev_priv.dev->bus_state == ERROR_PASSIVE) {
				s_mcp2515_dev_priv.dev->bus_state = ERROR_ACTIVE;
				p_err->dev_code = 0;
			} else {
				// bus off state
				s_mcp2515_dev_priv.dev->bus_state = ERROR_PASSIVE;
				p_err->dev_code = 1;
			}
			// error warning
		} else if ((err & _BV(MCP_TXWAR)) || (err & _BV(MCP_RXWAR))) {
			p_err->error_code = CAN_ERROR_WARNING;
			p_err->dev_buffer = 0;
			p_err->dev_code = 0;
		}

		// need to clear the RX1OVR and RX0OVR flags if set also
		// clear the error interrupt flag
		mcp2515_bit_modify(MCP_CANINTF, _BV(MCP_ERRIF), 0);
	}
out_of_here:
	return;
}

