#include "astar.h"
#include "common/zhash.h"
#include "common/zmaxheap.h"

typedef struct astar_node {
    void* data;
    float g; // Cost to get to this node.
    float f; // Heuristic cost remaining.
    struct astar_node* parent;
} astar_node_t;

astar_node_t* astar_node_create(void* data, float g, float f, astar_node_t* parent) {
    astar_node_t* node = malloc(sizeof(astar_node_t));
    node->data = data;
    node->g = g;
    node->f = f;
    node->parent = parent;
    return node;
}

zarray_t* reverse_parent_nodes(astar_node_t* node) {
    if (node == NULL) {
        return zarray_create(sizeof(void*));
    }
    zarray_t* ret = reverse_parent_nodes(node->parent);
    printf("%d\n", ((int*) node->data)[0]);
    zarray_add(ret, &node->data);
    return ret;
}

zarray_t* astar(
        int start_count,
        astar_node_t** start_nodes,
        int (*is_goal)(void* user, void* data),
        float (*edge_cost)(void* user, void* d1, void* d2),
        float (*heuristic_cost)(void* user, void* data),
        zarray_t* (*expand_node)(void* user, void* data),
        void* user) {
    printf("Starting astar\n");
    zmaxheap_t* heap = zmaxheap_create(sizeof(astar_node_t*));
    for (int i = 0; i < start_count; i++) {
        zmaxheap_add(heap, &(start_nodes[i]), -start_nodes[i]->g - start_nodes[i]->f);
    }
    zhash_t* visited = zhash_create(sizeof(void*), sizeof(astar_node_t*), zhash_ptr_hash, zhash_ptr_equals);

    astar_node_t* cur_node;
    while (zmaxheap_remove_max(heap, &cur_node, NULL)) {
        if (zhash_contains(visited, &(cur_node->data))) {
            free(cur_node);
            continue;
        }
        if (is_goal(user, cur_node->data)) {
            break;
        }
        zhash_put(visited, &(cur_node->data), &cur_node, NULL, NULL);
        printf("%d ->", ((int*) cur_node->data)[0]);
        zarray_t* next_nodes_data = expand_node(user, cur_node->data);
        for (int i = 0; i < zarray_size(next_nodes_data); i++) {
            void* next_data;
            zarray_get(next_nodes_data, i, &next_data);
            printf(" %d", ((int*) next_data)[0]);

            if (zhash_contains(visited, &next_data)) {
                continue;
            }

            astar_node_t* next_node = astar_node_create(
                    next_data,
                    cur_node->g + edge_cost(user, cur_node->data, next_data),
                    heuristic_cost(user, next_data),
                    cur_node);
            zmaxheap_add(heap, &next_node, -next_node->g - next_node->f);
        }
        printf("\n");
    }
    // TODO free all the nodes we created.

    // reconstruct path as list of node_t* and return
    printf("Path is:\n");
    zarray_t* ret = reverse_parent_nodes(cur_node);
    free(cur_node);
    zhash_vmap_values(visited, free);
    zhash_destroy(visited);
    zmaxheap_destroy(heap);
    return ret;
}

