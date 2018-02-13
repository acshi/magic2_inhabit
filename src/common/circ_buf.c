#include "circ_buf.h"

#include <stdlib.h>
#include <math.h>

circ_buf_t *circ_buf_create(int n)
{
    circ_buf_t *buf = calloc(1, sizeof(circ_buf_t));
    buf->n = n;
    buf->data = calloc(n, sizeof(float));
    buf->start = 0;
    buf->tail = 0;
    return buf;
}

void circ_buf_destroy(circ_buf_t *buf)
{
    free(buf->data);
    free(buf);
}

void circ_buf_push(circ_buf_t *buf, float val)
{
    int next_tail = (buf->tail + 1) % buf->n;
    if (buf->start == next_tail) {
        // full, replacing an old element
        buf->data[buf->tail] = val;
        buf->start = (buf->start + 1) % buf->n;
        buf->tail = next_tail;
    } else {
        buf->data[buf->tail] = val;
        buf->tail = next_tail;
    }
}

float circ_buf_pop(circ_buf_t *buf)
{
    if (circ_buf_empty(buf)) {
        return NAN;
    }
    float val = buf->data[buf->start];
    buf->start = (buf->start + 1) % buf->n;
    return val;
}

float circ_buf_top(circ_buf_t *buf)
{
    if (circ_buf_empty(buf)) {
        return NAN;
    }
    return buf->data[buf->start];
}

float circ_buf_last(circ_buf_t *buf)
{
    if (circ_buf_empty(buf)) {
        return NAN;
    }
    return buf->data[buf->tail];
}

float circ_buf_at(circ_buf_t *buf, int i)
{
    return buf->data[(buf->start + i) % buf->n];
}

bool circ_buf_empty(circ_buf_t *buf)
{
    return buf->start == buf->tail;
}
