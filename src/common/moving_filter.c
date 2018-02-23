#include "moving_filter.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>

moving_filter_t *moving_filter_create(int n)
{
    if (n <= 1) {
        return NULL;
    }

    moving_filter_t *f = calloc(1, sizeof(moving_filter_t));
    f->n = n;
    f->buff = circ_buf_create(n);
    return f;
}

void moving_filter_destroy(moving_filter_t *f)
{
    circ_buf_destroy(f->buff);
    free(f);
}

float moving_filter_march(moving_filter_t *f, float new_val) {
    float oldest_val = circ_buf_first(f->buff);
    bool buff_full = circ_buf_len(f->buff) == f->n;
    circ_buf_push(f->buff, new_val);

    if (buff_full) {
        f->current_sum -= oldest_val;
    }
    f->current_sum += new_val;

    return f->current_sum / circ_buf_len(f->buff);
}
