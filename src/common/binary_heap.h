#pragma once

#include <stdbool.h>
#include "common/zarray.h"

typedef struct binary_heap {
    zarray_t *v;
    zarray_t *f;
    float (*to_float)(void *a); // defaults to min-heap, add negation to make max-heap
} binary_heap_t;

binary_heap_t *binary_heap_create(float (*to_float)(void *a));
void binary_heap_destroy(binary_heap_t *heap);

static inline bool binary_heap_empty(binary_heap_t *heap)
{
    return zarray_size(heap->v) <= 1;
}

void binary_heap_push(binary_heap_t *heap, void *el);
void binary_heap_pop(binary_heap_t *heap);
void *binary_heap_top(binary_heap_t *heap);

static inline int binary_heap_size(binary_heap_t *heap)
{
    return zarray_size(heap->v) - 1;
}
