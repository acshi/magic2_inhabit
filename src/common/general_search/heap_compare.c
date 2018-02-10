#include "common/zmaxheap.h"
#include "common/binary_heap.h"

#include <stdio.h>
#include <stdlib.h>
#include <time.h>

double seconds() {
    struct timespec now;
    if (clock_gettime(CLOCK_MONOTONIC, &now)) {
        fprintf(stderr, "Retrieving system time failed.\n");
        exit(1);
    }
    return now.tv_sec + (double)now.tv_nsec * 1e-9;
}

void test_zmaxheap(int n, int *vals)
{
    zmaxheap_t *heap = zmaxheap_create(sizeof(void*));
    for (int i = 0; i < n; i++) {
        int *vp = &vals[i];
        zmaxheap_add(heap, &vp, -vals[i]);
    }
    int *val_p;
    float val;
    while (zmaxheap_remove_max(heap, &val_p, &val)) {
        printf("%d\n", *(int*)val_p);
    }
    zmaxheap_destroy(heap);
}

bool greater(void *a, void *b)
{
    int val_a = *((int*)a);
    int val_b = *((int*)b);
    return val_a > val_b;
}

void test_binaryheap(int n, int *vals)
{
    binary_heap_t *heap = binary_heap_create(greater);
    for (int i = 0; i < n; i++) {
        binary_heap_push(heap, &vals[i]);
    }
    while (!binary_heap_empty(heap)) {
        printf("%d\n", *(int*)binary_heap_top(heap));
        binary_heap_pop(heap);
    }
    binary_heap_destroy(heap);
}

int main(int argc, const char **argv)
{
    int n = 10;
    int vals[n];
    for (int i = 0; i < n; i++) {
        vals[i] = rand() % 500;
        printf("%d\n", vals[i]);
    }
    printf("\n");

    double start = seconds();
    test_binaryheap(n, vals);
    printf("Binary heap took %.3f seconds\n\n", seconds() - start);

    start = seconds();
    test_zmaxheap(n, vals);
    printf("zmaxheap took %.3f seconds\n", seconds() - start);
}
