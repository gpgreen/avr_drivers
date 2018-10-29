/*------------------------------------------------*/
/* UART functions                                 */

#include "defs.h"
#include <stdio.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/setbaud.h>
#include "uart.h"
#include "uart_def.h"
#include "fifo.h"

/* ---------------------------------------------- */

static struct fifo txfifo, rxfifo;

static volatile uint8_t error;

/* Initialize UART */

void uart_init()
{
	error = 0;

    fifo_init(&txfifo);
    fifo_init(&rxfifo);

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
	_UCSRB = _BV(_RXCIE) | _BV(_RXEN) | _BV(_TXEN);
}

void uart_set_baudrate(uint8_t ubrrh, uint8_t ubrrl, int use_2x)
{
	// disable uart rx
	_UCSRB &= ~(_BV(_RXCIE) | _BV(_RXEN));
	// run until tx buffer empty..
	while (fifo_count(&txfifo) > 0);
	// disable tx
	_UCSRB &= ~(_BV(_TXEN) | _BV(_UDRIE));
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
	_UCSRB = _BV(_RXCIE) | _BV(_RXEN) | _BV(_TXEN);
}

/* Get a received character */

uint8_t uart_test ()
{
	return fifo_count(&rxfifo);
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
    // convert unix line endings to serial stream line endings
    if (c == '\n')
        fifo_put_safe(&txfifo, '\r');
    fifo_put_safe(&txfifo, c);
    // turn on the data register empty interrupt, this will load
    // char into the hardware
    _UCSRB |= _BV(_UDRIE);
	return 0;
}

int uart_getchar(FILE* stream)
{
    while(fifo_count(&rxfifo) == 0)
        ;
    return fifo_get_safe(&rxfifo);
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
    uint8_t byte = _UDR;
	fifo_put_unsafe(&rxfifo, byte);
	// check for errors
	uint8_t d = _UCSRA;
	if (d & (_BV(_FE)|_BV(_DOR)|_BV(_UPE)))
	{
		error = d & (_BV(_FE)|_BV(_DOR)|_BV(_UPE));
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
    uint8_t byte;
    if (fifo_get_unsafe(&txfifo, &byte) == FIFO_OK)
        _UDR = byte;
	if (fifo_count(&txfifo) == 0)
    {
        _UCSRA &= ~_BV(_UDRE);
		_UCSRB &= ~_BV(_UDRIE);
    }
}

