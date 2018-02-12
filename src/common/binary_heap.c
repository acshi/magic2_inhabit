#include "binary_heap.h"
#include <stdio.h>

binary_heap_t *binary_heap_create(float (*to_float)(void *a))
{
    binary_heap_t *heap = calloc(1, sizeof(binary_heap_t));
    heap->v = zarray_create(sizeof(void*));
    heap->f = zarray_create(sizeof(float));
    heap->to_float = to_float;
    int *val = NULL;
    zarray_add(heap->v, &val);
    float fval = 0;
    zarray_add(heap->f, &fval);
    return heap;
}

void binary_heap_destroy(binary_heap_t *heap)
{
    zarray_destroy(heap->v);
    zarray_destroy(heap->f);
    free(heap);
}

static void percolate_up(binary_heap_t *heap)
{
    int i = zarray_size(heap->v) - 1;
    void **v_data = (void**)heap->v->data;
    float *f_data = (float*)heap->f->data;
    float fb = f_data[i];
    while (i > 1) {
        int parent_i = i / 2;

        float fa = f_data[parent_i];
        // printf("Comparing at %d %d (%.1f) (%.1f) (%.1f) (%.1f)\n", i, parent_i, f_data[i], f_data[parent_i], fb, fa);
        if (fa > fb) {
            // printf("less: %.1f %.1f\n", f_data[i], f_data[parent_i]);
            void *a = v_data[parent_i];
            void *b = v_data[i];
            v_data[i] = a;
            v_data[parent_i] = b;
            f_data[i] = fa;
            f_data[parent_i] = fb;
            // printf("now: %.1f %.1f\n", f_data[i], f_data[parent_i]);
        } else {
            return;
        }
        i = parent_i;
    }
}

static int min_child_i(binary_heap_t *heap, int i, float *fchild)
{
    if (i * 2 + 1 >= zarray_size(heap->v)) {
        float *f_data = (float*)heap->f->data;
        *fchild = f_data[i * 2];
        return i * 2;
    }
    float *f_data = (float*)heap->f->data;
    float fa = f_data[i * 2 + 1];
    float fb = f_data[i * 2];
    if (fa > fb) {
        *fchild = fb;
        return i * 2;
    } else {
        *fchild = fa;
        return i * 2 + 1;
    }
}

static void percolate_down(binary_heap_t *heap)
{
    int i = 1;
    void **v_data = (void**)heap->v->data;
    float *f_data = (float*)heap->f->data;
    float fa = f_data[i];
    while (i * 2 < zarray_size(heap->v)) {
        float fb = 0;
        int child_i = min_child_i(heap, i, &fb);
        if (fa > fb) {
            // printf("less: %.1f %.1f\n", f_data[i], f_data[child_i]);
            void *a = v_data[i];
            void *b = v_data[child_i];
            v_data[child_i] = a;
            v_data[i] = b;
            f_data[child_i] = fa;
            f_data[i] = fb;
            // printf("now: %.1f %.1f\n", f_data[i], f_data[child_i]);
        } else {
            return;
        }
        i = child_i;
    }
}

static inline void fast_v_zadd(zarray_t *za, void *v)
{
    zarray_ensure_capacity(za, za->size + 1);
    void **v_data = (void**)za->data;
    v_data[za->size] = v;
    za->size++;
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

static inline void fast_v_zpop_shuffle(zarray_t *za)
{
    void **v_data = (void**)za->data;
    v_data[1] = v_data[za->size - 1];
    za->size--;
}

void binary_heap_push(binary_heap_t *heap, void *el)
{

    fast_v_zadd(heap->v, el);
    float fval = heap->to_float(el);
    fast_f_zadd(heap->f, fval);
    percolate_up(heap);
}

void binary_heap_pop(binary_heap_t *heap)
{
    fast_v_zpop_shuffle(heap->v);
    fast_f_zpop_shuffle(heap->f);
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
