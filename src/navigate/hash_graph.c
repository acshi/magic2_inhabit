#include "common/zhash.h"
#include "common/zset.h"
#include "hash_graph.h"

// DEBUG
#include <stdio.h>


// A graph with a hashmap of node ids to node pointers.

typedef struct hash_graph {
    zhash_t* node_map; // Map of node id to node pointer.
} hash_graph_t;

hash_graph_t* hg_create() {
    hash_graph_t* graph = malloc(sizeof(hash_graph_t));
    graph->node_map = zhash_create(sizeof(int), sizeof(node_t*), zhash_int_hash, zhash_int_equals);
    return graph;
}

node_t* hg_add_node(hash_graph_t* hg, int id) {
    node_t* prev_node = hg_get_node(hg, id);
    if (prev_node != NULL) {
        return prev_node;
    }
    node_t* node = malloc(sizeof(node_t));
    node->id = id;
    node->edges = zarray_create(sizeof(node_t*));
    zhash_put(hg->node_map, &id, &node, NULL, NULL);
    return node;
}

int hg_has_edge(hash_graph_t* hg, node_t* n1, node_t* n2) {
    return zarray_contains(n1->edges, &n2);
}

void hg_add_edge(hash_graph_t* hg, node_t* n1, node_t* n2) {
    if (n1 == NULL || n2 == NULL) {
        return;
    }
    if (hg_has_edge(hg, n1, n2)) {
        return;
    }
    zarray_add(n1->edges, &n2);
    zarray_add(n2->edges, &n1);
}

node_t* hg_get_node(hash_graph_t* hg, int id) {
    node_t* node = NULL;
    zhash_get(hg->node_map, &id, &node);
    return node;
}

void hg_traverse_graph(hash_graph_t* hg, void (*visit_node)(void* user, node_t* node), void (*visit_edge)(void* user, node_t* n1, node_t* n2), void* user) {
    zset_t* visited = zset_create(sizeof(int), zhash_int_hash, zhash_int_equals);
    zhash_iterator_t zit;
    zhash_iterator_init(hg->node_map, &zit);
    node_t* node;
    while(zhash_iterator_next(&zit, NULL, &node)) {
        zset_add(visited, &node->id, NULL);
        if (visit_node) {
            visit_node(user, node);
        }
        if (visit_edge) {
            for (int i = 0; i < zarray_size(node->edges); i++) {
                node_t* node2;
                zarray_get(node->edges, i, &node2);
                if (!zset_contains(visited, &node2->id)) {
                    visit_edge(user, node, node2);
                }
            }
        }
    }
}
