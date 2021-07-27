/*------------------------------------------------*/
/* FIFO functions                                 */
#ifndef FIFO_H_
#define FIFO_H_

#include "defs.h"
#include <string.h>
#include <avr/io.h>
#include <util/atomic.h>

// Return values
#define FIFO_OK 0
#define FIFO_EMPTY -1
#define FIFO_FULL -2

// this must be defined in defs.h
//#define FIFO_SIZE 64

struct fifo {
	volatile uint8_t idx_w;
	volatile uint8_t idx_r;
	volatile uint8_t count;
    uint8_t bufsize;
	uint8_t* buf;
};

/* fifo_init
 * call if needed to initialize fifo
 */
inline void fifo_init(struct fifo* fifo, uint8_t buffer_size,
                      uint8_t* use_this_buffer)
{
 	fifo->idx_r = 0;
	fifo->idx_w = 0;
	fifo->count = 0;
    fifo->bufsize = buffer_size;
    fifo->buf = use_this_buffer;
    memset(fifo->buf, 0, fifo->bufsize);
}

/* 
 * fifo_count
 * returns number of bytes in the fifo
 */
inline uint8_t fifo_count (struct fifo* fifo)
{
	return fifo->count;
}

/*
 * fifo_put_safe
 * place a byte in the fifo. If the fifo is full, it will spin
 * until an interrupt removes enough bytes to allow space for
 * the new byte. Safe to use with only 1 concurrent reader or
 * writer.
 */
inline void fifo_put_safe(struct fifo* fifo, uint8_t c)
{
	while (fifo->count >= fifo->bufsize)
        ;
	ATOMIC_BLOCK(ATOMIC_FORCEON) 
	{
        fifo->buf[fifo->idx_w] = c;
		++fifo->count;
        if (++fifo->idx_w >= fifo->bufsize)
            fifo->idx_w = 0;
	}
}

/*
 * fifo_put_safe_unblocking
 * place a byte in the fifo. Returns whether fifo was full or not
 * Safe to use with only 1 concurrent reader or writer.
 */
inline uint8_t fifo_put_safe_unblocking(struct fifo* fifo, uint8_t c)
{
    uint8_t retval = FIFO_FULL;
	ATOMIC_BLOCK(ATOMIC_FORCEON) 
	{
        if (fifo->count < fifo->bufsize) {
            fifo->buf[fifo->idx_w] = c;
            ++fifo->count;
            if (++fifo->idx_w >= fifo->bufsize)
                fifo->idx_w = 0;
            retval = FIFO_OK;
        }
	}
    return retval;
}

/*
 * fifo_put_unsafe
 * place a byte in the fifo. 
 * Intended to be used when interrupts are already disabled
 * Returns FIFO_FULL if no space left
 * Returns FIFO_OK when arg byte is copied to FIFO
 */
inline int fifo_put_unsafe(struct fifo* fifo, uint8_t c)
{
	uint8_t i;

	i = fifo->idx_w;
	if (fifo->count >= fifo->bufsize)
		return FIFO_FULL;
 	fifo->buf[i++] = c;
	++fifo->count;
	if (i >= fifo->bufsize)
		i = 0;
	fifo->idx_w = i;
	return FIFO_OK;
}

/*
 * fifo_get_safe
 * get a byte from the fifo
 * Will spin if FIFO is empty until interrupt puts a byte in the FIFO
 * safe to use with only 1 concurrent reader and writer.
 * Returns byte from FIFO
 */
inline uint8_t fifo_get_safe(struct fifo* fifo)
{
	uint8_t d;

	while (fifo->count == 0)
        ;
	ATOMIC_BLOCK(ATOMIC_FORCEON) 
	{
        d = fifo->buf[fifo->idx_r];
		--fifo->count;
        if (++fifo->idx_r >= fifo->bufsize)
            fifo->idx_r = 0;
	}

	return d;
}

/*
 * fifo_get_safe_unblocking
 * get a byte from the fifo
 * safe to use with only 1 concurrent reader and writer.
 * Returns true if byte retrieved, false if not
 */
inline uint8_t fifo_get_safe_unblocking(struct fifo* fifo, uint8_t *ch)
{
    uint8_t retval = FIFO_EMPTY;
	ATOMIC_BLOCK(ATOMIC_FORCEON) 
	{
        if (fifo->count != 0)
        {
            *ch = fifo->buf[fifo->idx_r];
            --fifo->count;
            if (++fifo->idx_r >= fifo->bufsize)
                fifo->idx_r = 0;
            retval = FIFO_OK;
        }
	}

	return retval;
}

/*
 * fifo_get_unsafe
 * get a byte from the fifo
 * Intended to be used when interrupts are already disabled
 * Returns FIFO_EMPTY if fifo has no bytes,
 * Returns FIFO_OK and byte is copied to byte arg
 */
inline int fifo_get_unsafe(struct fifo* fifo, uint8_t* byte)
{
	uint8_t i;

	i = fifo->idx_r;
	if (fifo->count == 0)
		return FIFO_EMPTY;
	*byte = fifo->buf[i++];
	--fifo->count;
	if (i >= fifo->bufsize)
		i = 0;
	fifo->idx_r = i;

	return FIFO_OK;
}

#endif
