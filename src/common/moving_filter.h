#pragma once

#include <stdbool.h>
#include "circ_buf.h"

typedef struct moving_target {
    int n;
    circ_buf_t *buff;
    float current_sum;
} moving_filter_t;

moving_filter_t *moving_filter_create(int n);
void moving_filter_destroy(moving_filter_t *f);
float moving_filter_march(moving_filter_t *f, float newVal);
