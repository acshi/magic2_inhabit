#include "april_graph/april_graph.h"
#include "astar.h"
#include "common/doubles.h"
#include "common/floats.h"
#include "tm_planner.h"


typedef struct state {
    hash_graph_t* topo_map;
    zarray_t* ag_nodes;
    double* start_xy;
    double start_dist;
    node_t* start1;
    node_t* start2;

    double* goal_xy;
    double goal_dist;
    node_t* goal1;
    node_t* goal2;
} state_t;


void tm_find_start_and_goal_edges(void* user, node_t* n1, node_t* n2) {
    state_t* state = user;

    april_graph_node_t* ag_node1;
    april_graph_node_t* ag_node2;
    zarray_get(state->ag_nodes, n1->id, &ag_node1);
    zarray_get(state->ag_nodes, n2->id, &ag_node2);

    double edge[2];
    doubles_subtract(ag_node2->state, ag_node1->state, 2, edge);
    double edge_length = doubles_magnitude(edge, 2);

    double rooted_start[2];
    doubles_subtract(state->start_xy, ag_node1->state, 2, rooted_start);

    double dist_along_edge = doubles_dot(rooted_start, edge, 2)/edge_length;
    if (dist_along_edge < 0) {
        double dist = doubles_distance(state->start_xy, ag_node1->state, 2);
        if (dist < state->start_dist) {
            state->start1 = n1;
            state->start2 = NULL;
            state->start_dist = dist;
        }
    } else if (dist_along_edge > edge_length) {
        double dist = doubles_distance(state->start_xy, ag_node2->state, 2);
        if (dist < state->start_dist) {
            state->start1 = NULL;
            state->start2 = n2;
            state->start_dist = dist;
        }
    } else {
        doubles_scale(dist_along_edge/edge_length, edge, 2, edge);
        double dist = doubles_distance(rooted_start, edge, 2);
        if (dist < state->start_dist) {
            state->start1 = n1;
            state->start2 = n2;
            state->start_dist = dist;
        }
    }

    // Warning this is the same code as for the start edge just copy pasted.
    doubles_subtract(ag_node2->state, ag_node1->state, 2, edge);
    edge_length = doubles_magnitude(edge, 2);

    double rooted_goal[2];
    doubles_subtract(state->goal_xy, ag_node1->state, 2, rooted_goal);

    dist_along_edge = doubles_dot(rooted_goal, edge, 2)/edge_length;
    if (dist_along_edge < 0) {
        double dist = doubles_distance(state->goal_xy, ag_node1->state, 2);
        if (dist < state->goal_dist) {
            state->goal1 = n1;
            state->goal2 = NULL;
            state->goal_dist = dist;
        }
    } else if (dist_along_edge > edge_length) {
        double dist = doubles_distance(state->goal_xy, ag_node2->state, 2);
        if (dist < state->goal_dist) {
            state->goal1 = NULL;
            state->goal2 = n2;
            state->goal_dist = dist;
        }
    } else {
        doubles_scale(dist_along_edge/edge_length, edge, 2, edge);
        double dist = doubles_distance(rooted_goal, edge, 2);
        if (dist < state->goal_dist) {
            state->goal1 = n1;
            state->goal2 = n2;
            state->goal_dist = dist;
        }
    }
}


int is_goal(void* user, void* data) {
    state_t* state = user;
    return state->goal1 == data || state->goal2 == data;
}

float edge_cost(void* user, void* d1, void* d2) {
    state_t* state = user;
    int id1 = ((node_t*) d1)->id;
    int id2 = ((node_t*) d2)->id;
    april_graph_node_t* ag_node1;
    april_graph_node_t* ag_node2;
    zarray_get(state->ag_nodes, id1, &ag_node1);
    zarray_get(state->ag_nodes, id2, &ag_node2);
    return (float) doubles_distance(ag_node1->state, ag_node2->state, 2);
}

float heuristic_cost(void* user, void* data) {
    state_t* state = user;
    int id = ((node_t*) data)->id;
    april_graph_node_t* ag_node;
    zarray_get(state->ag_nodes, id, &ag_node);
    return (float) doubles_distance(ag_node->state, state->goal_xy, 2);
}

zarray_t* expand_node(void* user, void* data) {
    return ((node_t*) data)->edges;
}

// Here we find the start nodes, the goal nodes, and then use A* to plan
// between them. If the start (or goal) is closer to an edge than any one node
// we consider the nodes at both ends of that edge as start (or goal) nodes.
zarray_t* tm_plan_path(hash_graph_t* topo_map, zarray_t* ag_nodes, double start_xy[2], double goal_xy[2]) {
    state_t* state = malloc(sizeof(state_t));
    state->topo_map = topo_map;
    state->ag_nodes = ag_nodes;
    state->start_xy = start_xy;
    state->goal_xy = goal_xy;
    state->start_dist = 1000;
    state->goal_dist = 1000;

    hg_traverse_graph(topo_map, NULL, tm_find_start_and_goal_edges, state);

    // Short-circuit if we don't need to plan through any nodes.
    if (state->start1 == state->goal1 && state->start2 == state->goal2) {
        return zarray_create(sizeof(node_t*));
    }

    int start_nodes_length;
    astar_node_t* start_nodes[2];
    if (state->start2 == NULL) {
        printf("Start node: %d\n", state->start1->id);
        start_nodes_length = 1;
        start_nodes[0] = astar_node_create(state->start1, (float) state->start_dist, heuristic_cost(state, state->start1), NULL);
    } else {
        printf("Start nodes: %d %d\n", state->start1->id, state->start2->id);
        start_nodes_length = 2;
        april_graph_node_t* ag_node1;
        april_graph_node_t* ag_node2;
        zarray_get(state->ag_nodes, state->start1->id, &ag_node1);
        zarray_get(state->ag_nodes, state->start2->id, &ag_node2);
        float start1_dist = (float) doubles_distance(start_xy, ag_node1->state, 2);
        float start2_dist = (float) doubles_distance(start_xy, ag_node2->state, 2);
        start_nodes[0] = astar_node_create(state->start1, start1_dist, heuristic_cost(state, state->start1), NULL);
        start_nodes[1] = astar_node_create(state->start2, start2_dist, heuristic_cost(state, state->start2), NULL);
    }
    zarray_t* ret = astar(start_nodes_length, start_nodes, is_goal, edge_cost, heuristic_cost, expand_node, (void*) state);
    free(state);
    return ret;
}

