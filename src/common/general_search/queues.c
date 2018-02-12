#include "queues.h"
#include "common/binary_heap.h"
#include "common/zstack.h"
#include "common/zqueue.h"

#include <stdio.h>
#include <stdlib.h>

static float to_float(void *a) {
    return ((gen_search_node_t*)a)->ordering_cost;
}

void *priority_queue_make() {
    return binary_heap_create(to_float);
}

void priority_queue_destroy(void *q) {
    binary_heap_destroy((binary_heap_t*)q);
}

void priority_queue_add(void *q, void *value) {
    binary_heap_push((binary_heap_t*)q, value);
}

bool priority_queue_is_empty(void *q) {
    return binary_heap_empty((binary_heap_t*)q);
}

void *priority_queue_remove_first(void *q) {
    binary_heap_t *queue = (binary_heap_t*)q;
    if (binary_heap_empty(queue)) {
        return NULL;
    }
    void *element = binary_heap_top(queue);
    binary_heap_pop(queue);
    return element;
}

void populate_with_priority_queue(general_search_problem_t *p) {
    p->queue_make = priority_queue_make;
    p->queue_destroy = priority_queue_destroy;
    p->queue_add = priority_queue_add;
    p->queue_is_empty = priority_queue_is_empty;
    p->queue_remove_first = priority_queue_remove_first;
}

void *fifo_queue_make() {
    return zqueue_create(sizeof(void*));
}

void fifo_queue_destroy(void *q) {
    zqueue_destroy(q);
}

void fifo_queue_add(void *q, void *value) {
    zqueue_add_back((zqueue_t*)q, value);
}

bool fifo_queue_is_empty(void *q) {
    return zqueue_size((zqueue_t*)q) > 0;
}

void *fifo_queue_remove_first(void *q) {
    zqueue_t *queue = (zqueue_t*)q;
    if (fifo_queue_is_empty(q)) {
        return NULL;
    }
    void *element;
    zqueue_pop(queue, &element);
    return element;
}

void populate_with_fifo(general_search_problem_t *p) {
    p->queue_make = fifo_queue_make;
    p->queue_destroy = fifo_queue_destroy;
    p->queue_add = fifo_queue_add;
    p->queue_is_empty = fifo_queue_is_empty;
    p->queue_remove_first = fifo_queue_remove_first;
}

void *lifo_queue_make() {
    return zstack_create(sizeof(void*));
}

void lifo_queue_destroy(void *q) {
    zstack_destroy((zstack_t*)q);
}

void lifo_queue_add(void *q, void *value) {
    zstack_push((zstack_t*)q, value);
}

bool lifo_queue_is_empty(void *q) {
    return zstack_empty((zstack_t*)q);
}

void *lifo_queue_remove_first(void *q) {
    zstack_t *queue = (zstack_t*)q;
    if (zstack_empty(queue)) {
        return NULL;
    }
    void *element;
    zstack_pop(queue, &element);
    return element;
}

void populate_with_lifo(general_search_problem_t *p) {
    p->queue_make = lifo_queue_make;
    p->queue_destroy = lifo_queue_destroy;
    p->queue_add = lifo_queue_add;
    p->queue_is_empty = lifo_queue_is_empty;
    p->queue_remove_first = lifo_queue_remove_first;
}
