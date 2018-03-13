#pragma once
#include "common/zarray.h"

typedef struct astar_node astar_node_t;
astar_node_t* astar_node_create(void* data, float g, float f, astar_node_t* parent);
zarray_t* astar(
        int start_count,
        astar_node_t** start_nodes,
        int (*is_goal)(void* user, void* data),
        float (*edge_cost)(void* user, void* d1, void* d2),
        float (*heuristic_cost)(void* user, void* data),
        zarray_t* (*expand_node)(void* user, void* data), 
        void* user);

