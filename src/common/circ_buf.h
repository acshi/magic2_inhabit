#pragma once

#include <stdbool.h>

typedef struct circ_buf {
    int start;
    int tail;
    int n;
    float *data;
} circ_buf_t;

circ_buf_t *circ_buf_create(int n);
void circ_buf_destroy(circ_buf_t *buf);

void circ_buf_push(circ_buf_t *buf, float val);
float circ_buf_pop(circ_buf_t *buf); // NAN if empty
float circ_buf_top(circ_buf_t *buf);
float circ_buf_last(circ_buf_t *buf);
bool circ_buf_empty(circ_buf_t *buf);
