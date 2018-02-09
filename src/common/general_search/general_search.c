#include "general_search.h"

#include <stdio.h>
#include <stdlib.h>

#define node_t gen_search_node_t
#define problem_t general_search_problem_t

node_t *make_node(problem_t *p, node_t *parent, void *state, int32_t action) {
    node_t *node = (node_t*)calloc(1, sizeof(node_t));
    node->state = state;
    node->parent = parent;
    node->action = action;
    node->depth = parent ? parent->depth + 1 : 0;
    node->path_cost = parent ? parent->path_cost + p->step_cost(node, action) : 0;
    node->ordering_cost = p->ordering_cost(node);
    node->n_alive_children = 0;
    if (parent) {
        parent->n_alive_children++;
    }
    //printf("\tMade node with path_cost: %.0f depth: %d value: %d\n", node->path_cost, node->depth, *(int*)(node->state));
    return node;
}

void delete_node(problem_t *p, node_t *node) {
    node_t *parent = node->parent;
    //printf("Deleting node with path_cost: %.0f depth: %d value: %d\n", node->path_cost, node->depth, *(int*)(node->state));
    p->destroy_state(node->state);
    free(node);
    if (parent) {
        //free(state); // we don't want to free the initial_state which we did not create
        parent->n_alive_children--;
        if (parent->n_alive_children == 0) {
            //printf("\t");
            delete_node(p, parent);
        }
    }
}

float step_cost_one(node_t *node, int32_t action) {
    return 1;
}

float ordering_cost_simple(node_t *node) {
    return node->path_cost;
}

void blank_state_destroy(void *state) {

}

bool has_function(bool has_f, const char* name) {
    if (!has_f) {
        fprintf(stderr, "Error: Problem missing necessary %s function\n", name);
        return false;
    }
    return true;
}

void clear_queue(problem_t *p, void *q) {
    node_t *node = NULL;
    while((node = (node_t*)p->queue_remove_first(q))) {
        delete_node(p, node);
    }
}

// Checks the problem is correctly filled out
// provides some default implementations
bool check_problem(problem_t *p) {
    if (!p) {
        fprintf(stderr, "Error: Problem is Null\n");
        return false;
    }
    if (!p->step_cost) {
        p->step_cost = step_cost_one;
    }
    if (!p->ordering_cost) {
        p->ordering_cost = ordering_cost_simple;
    }
    if (!p->destroy_state) {
        p->destroy_state = blank_state_destroy;
    }
    if (p->use_iterative_depth && p->iterative_depth_init < 0) {
        fprintf(stderr, "Error: Problem iterative_depth_init must be non-negative\n");
        return false;
    }
    if (p->use_iterative_depth && p->iterative_depth_increment < 0) {
        fprintf(stderr, "Error: Problem iterative_depth_increment must be non-negative\n");
        return false;
    }
    if (!p->initial_state) {
        fprintf(stderr, "Error: Problem missing necessary initial_state\n");
        return false;
    }

    if (!p->allow_cycles && (
            !has_function(!!p->unordered_set_make, "unordered_set_make") ||
            !has_function(!!p->unordered_set_destroy, "unordered_set_destroy") ||
            !has_function(!!p->unordered_set_add, "unordered_set_add") ||
            !has_function(!!p->unordered_set_contains, "unordered_set_contains"))) {
        printf("Unordered set functions are required unless you allow cycles.");
        return false;
    }

    return has_function(!!p->is_goal, "is_goal") &&
            has_function(!!p->expand_state, "expand_state") &&
            has_function(!!p->next_new_state, "states_length") &&
            has_function(!!p->queue_make, "queue_make") &&
            has_function(!!p->queue_destroy, "queue_destroy") &&
            has_function(!!p->queue_add, "queue_add") &&
            has_function(!!p->queue_remove_first, "queue_remove_first");
}

void general_search_result_destroy(problem_t *p, node_t *result) {
    delete_node(p, result);
}

node_t *inner_tree_search(problem_t *p, bool *needs_depth_increase) {
    void *frontier = p->queue_make();
    p->queue_add(frontier, make_node(p, NULL, p->initial_state, 0));

    void *expanded_set;
    if (!p->allow_cycles) {
        expanded_set = p->unordered_set_make();
    }

    while(1) {
        if (p->queue_is_empty(frontier)) {
            clear_queue(p, frontier);
            p->queue_destroy(frontier);
            if (!p->allow_cycles) {
                p->unordered_set_destroy(expanded_set);
            }
            return NULL;
        }

        node_t *node = (node_t*)p->queue_remove_first(frontier);

        if (!p->allow_cycles && p->unordered_set_contains(expanded_set, node->state)) {
            // already expanded
            delete_node(p, node);
            continue;
        }

        //printf("Expanding node with path_cost: %.0f depth: %d value: %d\n", node->path_cost, node->depth, *(int*)(node->state));
        if (p->debugging) {
            printf("Expanding node with path_cost: %.2f depth: %d ", node->path_cost, node->depth);
        }
        p->expansion_count++;

        if (p->is_goal(node)) {
            clear_queue(p, frontier);
            p->queue_destroy(frontier);
            if (!p->allow_cycles) {
                p->unordered_set_destroy(expanded_set);
            }
            if (p->debugging) {
                printf("\n");
            }
            return node;
        }

        if (p->use_iterative_depth && node->depth >= p->_iterative_depth_limit) {
            delete_node(p, node);
            if (p->iterative_depth_increment > 0) {
                *needs_depth_increase = true;
            }
            if (p->debugging) {
                printf("\n");
            }
            continue;
        }

        void *expansion = p->expand_state(node->state);
        if (!p->allow_cycles) {
            p->unordered_set_add(expanded_set, node->state);
        }

        int32_t nodes_added = 0;
        void *new_state = NULL;
        int32_t new_action = -1;
        while (p->next_new_state(expansion, &new_state, &new_action)) {
            if (!p->allow_cycles && p->unordered_set_contains(expanded_set, new_state)) {
                continue;
            }
            node_t *new_node = make_node(p, node, new_state, new_action);
            p->queue_add(frontier, new_node);
            nodes_added++;
        }
        if (nodes_added == 0) {
            // leaf node will be forgotten
            delete_node(p, node);
        }

        if (p->debugging) {
            printf("\n");
        }
    }
}

node_t *tree_search(problem_t *p) {
    if (!check_problem(p)) {
        return NULL;
    }
    p->_iterative_depth_limit = p->iterative_depth_init;

    bool needs_depth_increase = false;
    node_t *result = inner_tree_search(p, &needs_depth_increase);
    while (!result && needs_depth_increase && p->_iterative_depth_limit < 500) {
        p->_iterative_depth_limit += p->iterative_depth_increment;
        printf("Increasing iterative-depth limit to %d\n", p->_iterative_depth_limit);
        result = inner_tree_search(p, &needs_depth_increase);
    }

    return result;
}
