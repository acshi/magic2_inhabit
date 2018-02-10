#pragma once

#include <stdbool.h>
#include "circ_buf.h"

typedef struct median_filter {
    int medianN;
    circ_buf_t *medianBuff;
    float *medianLows;
    float *medianHighs;
} median_filter_t;

// true on success, false on failure
bool median_filter_init(median_filter_t *f, int medianN);
float median_filter_march(median_filter_t *f, float newVal);
