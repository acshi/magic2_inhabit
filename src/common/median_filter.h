#pragma once

#include <stdbool.h>
#include "circ_buf.h"
#include "fbinary_heap.h"

typedef struct median_filter {
    int medianN;
    circ_buf_t *medianBuff;
    fbinary_heap_t *medianLows;
    fbinary_heap_t *medianHighs;
} median_filter_t;

median_filter_t *median_filter_create(int medianN);
void median_filter_destroy(median_filter_t *f);
float median_filter_march(median_filter_t *f, float newVal);
