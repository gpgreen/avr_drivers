#ifndef UART_DEF_H_
#define UART_DEF_H_

/* define which model of UART the device has:
 * MODEL_1
 * MODEL_1A
 * MODEL_2
 */

#if defined(__AVR_ATmega16__)
#define UART_MODEL_1 (1)
#elif defined(__AVR_ATtiny2313__)
#define UART_MODEL_1A (1)
#elif defined(__AVR_ATmega644__) || defined(__AVR_ATmega328__) || defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__) || defined(__AVR_AT90CAN32__)
#define UART_MODEL_2 (1)
#endif

/* MODEL_1 and MODEL_1A */
#if defined(UART_MODEL_1)

#define _UBRRH UBRRH
#define _UBRRL UBRRL
#define _UCSRA UCSRA
#define _UCSRB UCSRB
#define _UCSRC UCSRC

#define _U2X U2X
#define _UCSZ0 UCSZ0
#define _UCSZ1 UCSZ1
#define _RXCIE RXCIE
#define _RXEN RXEN
#define _TXEN TXEN
#define _UDRIE UDRIE
#define _FE FE
#define _DOR DOR
#define _UDR UDR

/* MODEL_1A has the UPE bit instead of PE bit */
#if defined(UART_MODEL_1A)
#define _UPE UPE
#define UART_8N1 (_BV(UMSEL)|_BV(UCSZ1)|_BV(UCSZ0))
#else
#define _UPE PE
#define UART_8N1 (_BV(URSEL)|_BV(UCSZ1)|_BV(UCSZ0))
#endif

#endif

/* MODEL_2
 * 
 * Some AVR devices have 2 serial ports
 * We can use the second one by
 * defining UART1, if this is not defined, we use the first port
 */
#if defined(UART1)

#define _UBRRH UBRR1H
#define _UBRRL UBRR1L
#define _UCSRA UCSR1A
#define _UCSRB UCSR1B
#define _UCSRC UCSR1C

#define _U2X U2X1
#define _UCSZ0 UCSZ10
#define _UCSZ1 UCSZ11
#define _RXCIE RXCIE1
#define _RXEN RXEN1
#define _TXEN TXEN1
#define _UDRIE UDRIE1
#define _FE FE1
#define _DOR DOR1
#define _UPE UPE1
#define _UDR UDR1

#define UART_8N1 (_BV(UCSZ11)|_BV(UCSZ10))

#else

/* UART0 */
#define _UBRRH UBRR0H
#define _UBRRL UBRR0L
#define _UCSRA UCSR0A
#define _UCSRB UCSR0B
#define _UCSRC UCSR0C

#define _U2X U2X0
#define _UCSZ0 UCSZ00
#define _UCSZ1 UCSZ01
#define _RXCIE RXCIE0
#define _RXEN RXEN0
#define _TXEN TXEN0
#define _UDRIE UDRIE0
#define _FE FE0
#define _DOR DOR0
#define _UPE UPE0
#define _UDR UDR0

#define UART_8N1 (_BV(UCSZ01)|_BV(UCSZ00))

#endif

#endif	// UART_DEF_H_
