#ifndef UART_H_
#define UART_H_

#include <stdio.h>
#include <stdint.h>

/* Defines needed in application programs
 * UART1 - if we are using the second serial port on the device instead
 * of the first
 * BAUD - this is used to calculate register values using header file
 * util/setbaud.h
 */

extern void uart_init(uint8_t txsz, uint8_t* txbuf, uint8_t rxsz, uint8_t* rxbuf);		/* Initialize UART and Flush FIFOs */
extern void uart_set_baudrate(uint8_t ubrrh, uint8_t ubrrl, int use_2x); /* set new baud rate */
extern uint8_t uart_test(void);	/* Check number of data in UART Rx FIFO */
extern uint8_t uart_error(void);   /* returns non-zero if there was an error */

extern int uart_putchar(char c, FILE* stream);
extern int uart_getchar(FILE* stream);

extern int uart_write_fifo_ovf(void);
extern int uart_read_fifo_ovf(void);

// unblocking versions, that don't use FILE stream
extern int uart_getchar_unblocking(uint8_t *c);
extern int uart_putchar_unblocking(char c);

#endif
