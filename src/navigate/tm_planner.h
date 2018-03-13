#pragma once
#include "common/zarray.h"
#include "hash_graph.h"

zarray_t* tm_plan_path(hash_graph_t* topo_map, zarray_t* ag_nodes, double start_xy[2], double goal_xy[2]);

