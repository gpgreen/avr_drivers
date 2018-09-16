/*------------------------------------------------*/
/* UART functions                                 */

#include "defs.h"
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/setbaud.h>
#include <stdlib.h>				/* for abs */
#include "uart.h"
#include "uart_def.h"

/* ---------------------------------------------- */

typedef struct _fifo {
	uint8_t	idx_w;
	uint8_t	idx_r;
	uint8_t	count;
	uint8_t buff[64];
} FIFO;


static volatile
FIFO txfifo, rxfifo;

static volatile uint8_t error;

/* Initialize UART */

void uart_init()
{
	error = 0;
	rxfifo.idx_r = 0;
	rxfifo.idx_w = 0;
	rxfifo.count = 0;
	txfifo.idx_r = 0;
	txfifo.idx_w = 0;
	txfifo.count = 0;

	// set baud rate
	_UBRRH = UBRRH_VALUE;
	_UBRRL = UBRRL_VALUE;
#if USE_2X
	_UCSRA |= _BV(_U2X);
#else
	_UCSRA &= ~_BV(_U2X);
#endif
	// set for 8 bit, 1 stop bit, no parity
	_UCSRC = UART_8N1;
	// enable tx, rx, receive complete interrupt
	_UCSRB = _BV(_RXCIE)|_BV(_RXEN)|_BV(_TXEN);
}

void uart_set_baudrate(uint8_t ubrrh, uint8_t ubrrl, int use_2x)
{
	// disable uart rx
	_UCSRB &= ~(_BV(_RXCIE)|_BV(_RXEN));
	// disable tx
	printf("rh:%x rl:%x 2x:%x\n", ubrrh, ubrrl, use_2x);
	// run until tx buffer empty..
	while (txfifo.count > 0);
	_UCSRB &= ~(_BV(_TXEN)|_BV(_UDRIE));
	if (use_2x) {
		_UBRRL = ubrrl;
		_UBRRH = ubrrh;
		_UCSRA |= _BV(_U2X);
	} else {
		_UBRRL = ubrrl;
		_UBRRH = ubrrh;
		_UCSRA &= ~_BV(_U2X);
	}
	// enable
	_UCSRB = _BV(_RXCIE)|_BV(_RXEN)|_BV(_TXEN);
}

/* Get a received character */

uint8_t uart_test ()
{
	return rxfifo.count;
}

uint8_t uart_error()
{
	uint8_t code = 0;
	if (error)
	{
		code = error;
		error = 0;
	}
	return code;
}

int uart_putchar(char c, FILE* unused)
{
	uint8_t i;

	i = txfifo.idx_w;
	while(txfifo.count >= sizeof(txfifo.buff));
	txfifo.buff[i++] = c;
	cli();
	txfifo.count++;
	_UCSRB |= _BV(_UDRIE);
	sei();
	if(i >= sizeof(txfifo.buff))
		i = 0;
	txfifo.idx_w = i;

	return 0;
}

int uart_getchar(FILE* stream)
{
	uint8_t d, i;


	i = rxfifo.idx_r;
	while(rxfifo.count == 0);
	d = rxfifo.buff[i++];
	cli();
	rxfifo.count--;
	sei();
	if(i >= sizeof(rxfifo.buff))
		i = 0;
	rxfifo.idx_r = i;

	return d;
}


/* UART RXC interrupt */

#if defined(__AVR_ATmega16__)
ISR(USART_RXC_vect)
#elif defined(__AVR_ATmega328__) || defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__) || defined(__AVR_ATtiny2313__)
ISR(USART_RX_vect)
#elif defined(__AVR_ATmega644__) || defined(__AVR_AT90CAN32__)
#if defined(UART1)
ISR(USART1_RX_vect)
#else
ISR(USART0_RX_vect)
#endif
#endif
{
	uint8_t d, n, i;

	// check for errors
	d = _UCSRA;
	if (d & (_BV(_FE)|_BV(_DOR)|_BV(_UPE)))
	{
		error = d & (_BV(_FE)|_BV(_DOR)|_BV(_UPE));
		return;
	}
	d = _UDR;
	n = rxfifo.count;
	if(n < sizeof(rxfifo.buff)) {
		rxfifo.count = ++n;
		i = rxfifo.idx_w;
		rxfifo.buff[i++] = d;
		if(i >= sizeof(rxfifo.buff))
			i = 0;
		rxfifo.idx_w = i;
	}
}


/* UART UDRE interrupt */

#if defined(__AVR_ATmega16__) || defined(__AVR_ATmega328__) || defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__) || defined(__AVR_ATtiny2313__)
ISR(USART_UDRE_vect)
#elif defined(__AVR_ATmega644__) || defined(__AVR_AT90CAN32__)
#if defined(UART1)
ISR(USART1_UDRE_vect)
#else
ISR(USART0_UDRE_vect)
#endif
#endif
{
	uint8_t n, i;


	n = txfifo.count;
	if(n) {
		txfifo.count = --n;
		i = txfifo.idx_r;
		_UDR = txfifo.buff[i++];
		if(i >= sizeof(txfifo.buff))
			i = 0;
		txfifo.idx_r = i;
	}
	if(n == 0)
		_UCSRB &= ~_BV(_UDRIE);
}

