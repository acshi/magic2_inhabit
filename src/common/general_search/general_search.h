#pragma once

#include <stdbool.h>
#include <stdint.h>

typedef struct node {
    void *state;
    struct node *parent;
    int32_t action;
    int32_t depth;
    float path_cost;
    float ordering_cost;
    int n_alive_children;
} gen_search_node_t;

typedef struct problem {
    void *initial_state;

    int32_t expansion_count;

    bool allow_cycles;
    bool debugging;

    bool use_iterative_depth;
    int32_t iterative_depth_init;
    int32_t iterative_depth_increment;
    int32_t _iterative_depth_limit; // internal, factor of the above

    bool (*is_goal)(gen_search_node_t *node);
    float (*step_cost)(gen_search_node_t *node, int32_t action);
    float (*ordering_cost)(gen_search_node_t *node);

    // Returns an expansion, for use with next_new_state
    void *(*expand_state)(void *state);
    // true when new_state and new_action are populated. false when no new states left
    // guarentees to not call again after it returns false (so cleanup can be performed)
    bool (*next_new_state)(void *expansion, void **new_state, int32_t *new_action);
    // optional for freeing state resources
    void (*destroy_state)(void *state);

    void *(*queue_make)();
    void (*queue_destroy)(void *q);
    void (*queue_add)(void *q, void *value);
    bool (*queue_is_empty)(void *q);
    void *(*queue_remove_first)(void *q);

    void *(*unordered_set_make)();
    void (*unordered_set_destroy)(void *s);
    void (*unordered_set_add)(void *s, void *value);
    bool (*unordered_set_contains)(void *s, void *value);
} general_search_problem_t;

gen_search_node_t *tree_search(general_search_problem_t *p);
void general_search_result_destroy(general_search_problem_t *p, gen_search_node_t *result);
