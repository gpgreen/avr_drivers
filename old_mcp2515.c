#include <avr/io.h>
#include <util/delay.h>
#include <util/delay_basic.h>
#include <util/atomic.h>
#include "old_mcp2515.h"
#include "old_mcp2515_def.h"
#include "spi.h"
#include "globals.h"
#include "driver.h"

// define the following to debug SPI comm's
// ----------------------------------------
/* #define MCPSPIDEBUG 1 */


// globals

static const char* k_name = "mcp2515:";

// can device
struct can_device mcp2515_dev = {
	.device_name = "mcp2515",
	.init_fn = mcp2515_init,
	.self_test_fn = mcp2515_self_test,
	.check_receive_fn = mcp2515_check_receive,
	.free_send_buffer_fn = mcp2515_get_next_free_tx_buf,
	.write_msg_fn = mcp2515_write_msg,
	.read_msg_fn = mcp2515_read_msg,
	.handle_int_fn = mcp2515_handle_interrupt,
};

// set CS low to select device
static void mcp2515_select(void)
{
    PORT_CANCS &= ~(_BV(P_CANCS));
	_delay_loop_1(2);
}

// set CS high to unselect device
static void mcp2515_unselect(void)
{
	_delay_loop_1(2);
    PORT_CANCS |= _BV(P_CANCS);
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

static uint8_t mcp2515_read_status(void)
{
	mcp2515_select();
	spi_transfer(MCP_SPI_READ_STATUS);
	uint8_t data = spi_transfer(0xff);
	// repeat the status 2nd byte
	spi_transfer(0xff);
	mcp2515_unselect();
#ifdef MCPDEBUG
	DEVICE_PRINT2("read_status:%x", data);
#endif

	return data;
}

/*
 * Check for received CAN messageso
 */
uint8_t mcp2515_check_receive(void)
{
#ifdef MCPDEBUG
	DEVICE_PRINT("check_receive");
#endif
	
	if (mcp2515_read_status() & (_BV(MCP_CANINTF_RX0IF)|_BV(MCP_CANINTF_RX1IF)))
	 	return CAN_MSGAVAIL;
	return CAN_NOMSG;
}

/*
 * Read MCP2515 receiver status
 */
uint8_t mcp2515_read_rx_status(void)
{
    uint8_t data;

	mcp2515_select();
   
    spi_transfer(MCP_SPI_RX_STATUS);
    data = spi_transfer(0xff);
   
    // read the status 2nd byte, don't care about
    // it's value coming or going
    spi_transfer(0xff);
   
	mcp2515_unselect();
   
#ifdef MCPDEBUG
	DEVICE_PRINT2("read_rx_status:%x", data);
#endif   

    return data;
}

/*
 * Initialize the MCP2515
 */
uint8_t mcp2515_init(can_init_t* settings)
{
	uint8_t cfg1, cfg2, cfg3;

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
		failed(1);
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
   
    // Activate the Tx,Rx Buffer Interrupts
    mcp2515_write_register(MCP_CANINTE,
		_BV(MCP_RX1IE)|_BV(MCP_RX0IE)|_BV(MCP_TX0IE)|_BV(MCP_TX1IE)|_BV(MCP_TX2IE));

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
	*g_mcp2515_dev_priv.ddr_port &= ~g_mcp2515_dev_priv.port_pin;
	*g_mcp2515_dev_priv.port |= g_mcp2515_dev_priv.port_pin;

    // setup interrupt for CAN
	*g_mcp2515_dev_priv.int_dir_reg |= g_mcp2515_dev_priv.int_dir_mask;
	*g_mcp2515_dev_priv.int_en_reg |= g_mcp2515_dev_priv.int_en_mask;

	return CAN_OK;
}

/*
 * Returns: CAN_OK, CAN_FAIL
 */
uint8_t mcp2515_self_test(void)
{
	// read the CANCTRL register, ensure that it matches setup
	uint8_t reg = mcp2515_read_register( MCP_CANCTRL );
	if (reg != (g_mcp2515_dev_priv.settings->loopback_on ? _BV(MCP_REQOP1) : 0))
		return CAN_FAIL;
	// read the CANSTAT register, ensure in normal mode
	reg = mcp2515_read_register( MCP_CANSTAT );
	// mask off bits other than mode (interrupts)
	// should be in normal operation mode
	uint8_t tstval = g_mcp2515_dev_priv.settings->loopback_on ? _BV(MCP_OPMOD1) : 0;
	if ((reg & (_BV(MCP_OPMOD0)|_BV(MCP_OPMOD1)|_BV(MCP_OPMOD2))) != tstval)
		return CAN_FAIL;
	// read the interrupt enable
	reg = mcp2515_read_register( MCP_CANINTE );
	if (reg != (_BV(MCP_RX0IE) | _BV(MCP_RX1IE) | _BV(MCP_TX0IE) | _BV(MCP_TX1IE) | _BV(MCP_TX2IE)))
		return CAN_FAIL;
	return CAN_OK;
}

uint8_t mcp2515_get_next_free_tx_buf(int* txbuf_n)
{
	*txbuf_n = 0x00;

    // Get MCP2515 status
	uint8_t status = mcp2515_read_status();
	// test the status byte to find a vacant tx buffer
    if (bit_is_clear(status, MCP_TXB0CNTRL_TXREQ)) {
		*txbuf_n = 0x00;
    }
    else if (bit_is_clear(status, MCP_TXB1CNTRL_TXREQ)) {
		*txbuf_n = 0x02;
    }
    else if (bit_is_clear(status, MCP_TXB2CNTRL_TXREQ)) {
		*txbuf_n = 0x04;
    }
    else {
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
void mcp2515_write_msg(int buffer_sidh_addr, const can_msg_t *p_message)
{
#ifdef MCPDEBUG
	DEVICE_PRINT2("write_msg:%x", buffer_sidh_addr);
#endif	
	mcp2515_select();
    spi_transfer(MCP_SPI_LOAD_TX0 | buffer_sidh_addr);
   
    // Standard ID
    spi_transfer((uint8_t) (p_message->id>>3));
    spi_transfer((uint8_t) (p_message->id<<5));
   
    // Extended ID
    spi_transfer(0x00);
    spi_transfer(0x00);
   
    uint8_t length = p_message->length;
   
    if (length > 8) {
        length = 8;
    }
   
    // is this "Remote Transmit Request" ?
    if (p_message->rtr)
    {
        // data len + RTR
        spi_transfer(_BV(MCP_RTR) | length);
    }
    else
    {
        // data len
        spi_transfer(length);
       
        // Data
        for (uint8_t i=0;i<length;i++) {
            spi_transfer(p_message->data[i]);
        }
    }
	mcp2515_unselect();

	mcp2515_start_transmit(buffer_sidh_addr);
	
}

void mcp2515_start_transmit(const uint8_t txbuf_n)
{
    /* CAN data ready to send
	   Using address from above, tell the chip ready to send */
	mcp2515_select();
    if (txbuf_n == 0x00) {
        spi_transfer(MCP_SPI_RTS_TX0 | 0x01);
    } else {
        spi_transfer(MCP_SPI_RTS_TX0 | txbuf_n);
    }
	mcp2515_unselect();
}

/*
 * read a CAN message
 */
uint8_t mcp2515_read_msg(can_msg_t *p_message)
{
	uint8_t rx_status = mcp2515_read_rx_status();
	if(!(rx_status & MCP_MSG_RECV_MASK))
		return CAN_NOMSG;
	
	uint8_t sidh_addr = 0;

	// find which buffer has the msg: rx buf 0, or rx buf 1
	if (bit_is_set(rx_status, MCP_MSG_RXB0))
		sidh_addr = MCP_SPI_READ_RX0;
	else if (bit_is_set(rx_status, MCP_MSG_RXB1))
		sidh_addr = MCP_SPI_READ_RX1;

#ifdef MCPDEBUG
	DEVICE_PRINT2("read_msg addr:%x", sidh_addr);
#endif

	mcp2515_select();
	spi_transfer(sidh_addr);

    // Standard ID
    p_message->id =  (uint16_t) spi_transfer(0xff) << 3;
    p_message->id |= (uint16_t) spi_transfer(0xff) >> 5;
   
    // extended ID
    spi_transfer(0xff);
    spi_transfer(0xff);
   
    // length 
    uint8_t length = spi_transfer(0xff) & 0x0f;
    p_message->length = length;
   
    // Read Data
    for (uint8_t i=0;i<length;i++) {
        p_message->data[i] = spi_transfer(0xff);
    }
   
	mcp2515_unselect();

    if (bit_is_set(rx_status,MCP_REMOTE_FRAME)) {
        p_message->rtr = 1;
    } else {
        p_message->rtr = 0;
    }
	return CAN_OK;
}

/*
 * handle interrupts, clearing flags
 * the current status is put into the arg status_flag
 * this clears transmission sent flags
 * so that tx buffers can be reused
 * rx flags are cleared by can_get_message instead
 */
uint8_t mcp2515_handle_interrupt(uint8_t* status_flag)
{
	ATOMIC_BLOCK(ATOMIC_FORCEON)
	{
		*status_flag = g_mcp2515_dev_priv.flag;
		g_mcp2515_dev_priv.flag = 0;
	}

	// if flag is set, we have an interrupt
	if (*status_flag) {

#ifdef MCPDEBUG
		DEVICE_PRINT("handle_interrupt");
#endif	

		// read the status
		*status_flag = mcp2515_read_status();

		uint8_t mask = 0;
		// set the mask for each interrupt flag found
		if (bit_is_set(*status_flag, MCP_CANINTF_TX0IF)) {
			mask |= _BV(MCP_TX0IF);
		}
		if (bit_is_set(*status_flag, MCP_CANINTF_TX1IF)) {
			mask |= _BV(MCP_TX1IF);
		}
		if (bit_is_set(*status_flag, MCP_CANINTF_TX2IF)) {
			mask |= _BV(MCP_TX2IF);
		}
		
		// send the command to clear interrupt flag(s)
		mcp2515_bit_modify(MCP_CANINTF, mask, 0);
	}

	return *status_flag ? CAN_INTERRUPT : CAN_NOINTERRUPT;
}

/*
 * Either a pin change interrupt, or an interrupt triggered by falling edge
 * depending on whether MCP2515_INT_VEC_ANY_CHANGE is defined
 */
ISR(MCP2515_INT_VECT)
{
#if defined(MCP2515_INT_VECT_ANY_CHANGE)
	// see if can interrupt pin is lo, ie we got a falling edge
	if (bit_is_clear(*g_mcp2515_dev_priv.pin, g_mcp2515_dev_priv.port_pin))
		g_mcp2515_dev_priv.flag = 1;
#else
	g_mcp2515_dev_priv.flag = 1;
#endif
}
