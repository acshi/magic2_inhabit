#pragma once
#include "common/zarray.h"

typedef struct hash_graph  hash_graph_t;
typedef struct node {
    int id;
    zarray_t* edges; // node_t*
} node_t;


hash_graph_t* hg_create();
node_t* hg_add_node(hash_graph_t* hg, int id);
node_t* hg_get_node(hash_graph_t* hg, int id);
void hg_traverse_graph(hash_graph_t* hg, void (*visit_node)(void* user, node_t* node), void (*visit_edge)(void* user, node_t* n1, node_t* n2), void* user);
//hg_has_edge(node_t* n1, node_t* n2);
void hg_add_edge(hash_graph_t* hg, node_t* n1, node_t* n2);

