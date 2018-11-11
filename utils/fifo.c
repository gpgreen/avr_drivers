#include "fifo.h"

extern inline void fifo_init(struct fifo* fifo, uint8_t buffer_size,
                             uint8_t* use_this_buffer);
extern inline uint8_t fifo_count (struct fifo* fifo);

extern inline void fifo_put_safe(struct fifo* fifo, uint8_t c);

extern inline int fifo_put_unsafe(struct fifo* fifo, uint8_t c);

extern inline uint8_t fifo_get_safe(struct fifo* fifo);

extern inline int fifo_get_unsafe(struct fifo* fifo, uint8_t* byte);

