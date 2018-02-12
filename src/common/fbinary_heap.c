#include "fbinary_heap.h"
#include <stdio.h>

fbinary_heap_t *fbinary_heap_create(bool is_max)
{
    fbinary_heap_t *heap = calloc(1, sizeof(fbinary_heap_t));
    heap->v = zarray_create(sizeof(float));
    heap->is_max = is_max;
    float val = 0;
    zarray_add(heap->v, &val); // unused element
    return heap;
}

void fbinary_heap_destroy(fbinary_heap_t *heap)
{
    zarray_destroy(heap->v);
    free(heap);
}

static void percolate_up(fbinary_heap_t *heap)
{
    int i = zarray_size(heap->v) - 1;
    float *v_data = (float*)heap->v->data;
    float b = v_data[i];
    while (i > 1) {
        int parent_i = i / 2;
        float a = v_data[parent_i];
        if (heap->is_max) {
            if (a < b) {
                v_data[i] = a;
                v_data[parent_i] = b;
            } else {
                return;
            }
        } else {
            if (a > b) {
                v_data[i] = a;
                v_data[parent_i] = b;
            } else {
                return;
            }
        }
        i = parent_i;
        b = a;
    }
}

static int min_child_i(fbinary_heap_t *heap, int i, float *vchild)
{
    if (i * 2 + 1 >= zarray_size(heap->v)) {
        float *v_data = (float*)heap->v->data;
        *vchild = v_data[i * 2];
        return i * 2;
    }
    float *v_data = (float*)heap->v->data;
    float a = v_data[i * 2 + 1];
    float b = v_data[i * 2];
    if (heap->is_max) {
        if (a < b) {
            *vchild = b;
            return i * 2;
        } else {
            *vchild = a;
            return i * 2 + 1;
        }
    } else {
        if (a > b) {
            *vchild = b;
            return i * 2;
        } else {
            *vchild = a;
            return i * 2 + 1;
        }
    }
}

static void percolate_down(fbinary_heap_t *heap)
{
    int i = 1;
    float *v_data = (float*)heap->v->data;
    float a = v_data[i];
    while (i * 2 < zarray_size(heap->v)) {
        float b = 0;
        int child_i = min_child_i(heap, i, &b);
        if (heap->is_max) {
            if (a < b) {
                v_data[child_i] = a;
                v_data[i] = b;
            } else {
                return;
            }
        } else {
            if (a > b) {
                v_data[child_i] = a;
                v_data[i] = b;
            } else {
                return;
            }
        }
        i = child_i;
        a = b;
    }
}

static inline void fast_f_zadd(zarray_t *za, float f)
{
    zarray_ensure_capacity(za, za->size + 1);
    float *f_data = (float*)za->data;
    f_data[za->size] = f;
    za->size++;
}

static inline void fast_f_zpop_shuffle(zarray_t *za)
{
    float *f_data = (float*)za->data;
    f_data[1] = f_data[za->size - 1];
    za->size--;
}

void fbinary_heap_push(fbinary_heap_t *heap, float el)
{
    fast_f_zadd(heap->v, el);
    percolate_up(heap);
}

void fbinary_heap_pop(fbinary_heap_t *heap)
{
    fast_f_zpop_shuffle(heap->v);
    percolate_down(heap);
}

float fbinary_heap_top(fbinary_heap_t *heap)
{
    if (fbinary_heap_empty(heap)) {
        fprintf(stderr, "Fatal Error: Called `top` on empty fbinary_heap\n");
        exit(1);
    }
    float *v_data = (float*)heap->v->data;
    return v_data[1];
}
