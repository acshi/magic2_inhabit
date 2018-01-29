#pragma once

#include <stdbool.h>
#include "common/zarray.h"

typedef struct binary_heap {
    zarray_t *v;
    bool (*greater)(void *a, void *b);
} binary_heap_t;

binary_heap_t *binary_heap_create(bool (*greater)(void *a, void *b));
void binary_heap_destroy(binary_heap_t *heap);
bool binary_heap_empty(binary_heap_t *heap);
void binary_heap_push(binary_heap_t *heap, void *el);
void binary_heap_pop(binary_heap_t *heap);
void *binary_heap_top(binary_heap_t *heap);
