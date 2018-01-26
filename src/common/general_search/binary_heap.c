#include "binary_heap.h"
#include <stdio.h>

binary_heap_t *binary_heap_create(bool (*greater)(void *a, void *b))
{
    binary_heap_t *heap = calloc(1, sizeof(binary_heap_t));
    heap->v = zarray_create(sizeof(void*));
    heap->greater = greater;
    int *val = NULL;
    zarray_add(heap->v, &val);
    return heap;
}

void binary_heap_destroy(binary_heap_t *heap)
{
    zarray_destroy(heap->v);
    free(heap);
}

bool binary_heap_empty(binary_heap_t *heap)
{
    return zarray_size(heap->v) <= 1;
}

static void percolate_up(binary_heap_t *heap)
{
    int i = zarray_size(heap->v) - 1;
    void **v_data = (void**)heap->v->data;
    while (i > 1) {
        int parent_i = i / 2;
        // printf("Comparing at %d %d\n", i, parent_i);
        void *a = v_data[parent_i];
        void *b = v_data[i];
        if (heap->greater(a, b)) {
            // printf("less: %d %d\n", this->v[i], this->v[parent_i]);
            v_data[i] = a;
            v_data[parent_i] = b;
            // printf("now: %d %d\n", this->v[i], this->v[parent_i]);
        } else {
            return;
        }
        i = parent_i;
    }
}

static int min_child_i(binary_heap_t *heap, int i)
{
    if (i * 2 + 1 >= zarray_size(heap->v)) {
        return i * 2;
    }
    void **v_data = (void**)heap->v->data;
    void *a = v_data[i * 2 + 1];
    void *b = v_data[i * 2];
    if (heap->greater(a, b)) {
        return i * 2;
    } else {
        return i * 2 + 1;
    }
}

static void percolate_down(binary_heap_t *heap)
{
    int i = 1;
    void **v_data = (void**)heap->v->data;
    while (i * 2 < zarray_size(heap->v)) {
        int child_i = min_child_i(heap, i);
        void *a = v_data[i];
        void *b = v_data[child_i];
        if (heap->greater(a, b)) {
            // printf("less: %d %d\n", this->v[i], this->v[parent_i]);
            v_data[child_i] = a;
            v_data[i] = b;
            // printf("now: %d %d\n", this->v[i], this->v[parent_i]);
        } else {
            return;
        }
        i = child_i;
    }
}

void binary_heap_push(binary_heap_t *heap, void *el)
{
    zarray_add(heap->v, &el);
    percolate_up(heap);
}

void binary_heap_pop(binary_heap_t *heap)
{
    zarray_remove_index(heap->v, 1, true);
    percolate_down(heap);
}

void *binary_heap_top(binary_heap_t *heap)
{
    if (binary_heap_empty(heap)) {
        fprintf(stderr, "Fatal Error: Called `top` on empty binary_heap\n");
        exit(1);
    }
    void **v_data = (void**)heap->v->data;
    return v_data[1];
}
