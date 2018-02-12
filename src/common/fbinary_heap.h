#pragma once

#include <stdbool.h>
#include "common/zarray.h"

typedef struct fbinary_heap {
    zarray_t *v;
    bool is_max;
} fbinary_heap_t;

fbinary_heap_t *fbinary_heap_create(bool is_max);
static inline fbinary_heap_t *fbinary_heap_min_create() {
    return fbinary_heap_create(false);
}
static inline fbinary_heap_t *fbinary_heap_max_create() {
    return fbinary_heap_create(true);
}

void fbinary_heap_destroy(fbinary_heap_t *heap);

static inline bool fbinary_heap_empty(fbinary_heap_t *heap)
{
    return zarray_size(heap->v) <= 1;
}

void fbinary_heap_push(fbinary_heap_t *heap, float el);
void fbinary_heap_pop(fbinary_heap_t *heap);
float fbinary_heap_top(fbinary_heap_t *heap);
