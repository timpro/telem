//
// ring.c -- Ring buffers
//
//  Copyright (c) 2012-2013 Andrew Payne <andy@payne.org>
//

#include <freedom.h>
#include "common.h"

inline void buf_reset(RingBuffer *buf, short size)
{
    buf->head = buf->tail = 0;
    buf->size = size;
}

inline short buf_len(const RingBuffer *buf)
{
    short len = buf->tail - buf->head;
    if (len < 0)
        len += buf->size;
    
    return len;
}

inline short buf_isfull(const RingBuffer *buf)
{
    return (buf_len(buf) + 1 == buf->size);
}

inline short buf_isempty(const RingBuffer *buf)
{
    return buf->head == buf->tail;
}

inline uint8_t buf_get_byte(RingBuffer *buf)
{
    short atomic = buf->head;
    const uint8_t item = buf->data[atomic++];

    if (atomic >= buf->size)         // Wrap
        buf->head = 0;
    else
	buf->head = atomic;

    return item;
}

inline void buf_put_byte(RingBuffer *buf, uint8_t val)
{
    short atomic = buf->tail;
    buf->data[atomic++] = val;
    if (atomic >= buf->size)
        buf->tail = 0;
    else
	buf->tail=atomic;
}
