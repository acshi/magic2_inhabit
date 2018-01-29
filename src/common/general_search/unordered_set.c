#include "unordered_set.h"
#include "common/zset.h"

static uint32_t hash(const void *a)
{
    uint64_t val = (uint64_t)*((void**)a);
    return (uint32_t)(val >> 32) ^ (uint32_t)val;
}

static int equals(const void *a, const void *b)
{
    return *((void**)a) == *((void**)b);
}

void *unordered_set_make()
{
    return zset_create(sizeof(void*), hash, equals);
}

void unordered_set_destroy(void *s)
{
    zset_destroy((zset_t*)s);
}

void unordered_set_add(void *s, void *value)
{
    zset_add((zset_t*)s, value, NULL);
}

bool unordered_set_contains(void *s, void *value)
{
    return zset_contains((zset_t*)s, value);
}

void populate_with_unordered_set(general_search_problem_t *p)
{
    p->unordered_set_make = unordered_set_make;
    p->unordered_set_destroy = unordered_set_destroy;
    p->unordered_set_add = unordered_set_add;
    p->unordered_set_contains = unordered_set_contains;
}
