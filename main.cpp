#include <stdlib.h>
#include <time.h>
#include "main.h"
#include "dataStructs.h"
#include "util.h"
#include <stdio.h>

point_t start, goal;
int max_num_of_nodes, num_of_obstacles;
int map_dim_x, map_dim_y;
int dist_to_grow;

node_t growFromNode(node_t *node, point_t direction, int dist_to_grow){
    int x_dist = direction.x - node->point.x;
    int y_dist = direction.y - node->point.y;
    float dist = euclideanDistance(node->point, direction);
    float dist_ratio = dist_to_grow / dist;

    node_t new_node;
    new_node.point.x = node->point.x + (x_dist * dist_ratio);
    new_node.point.y = node->point.y + (y_dist * dist_ratio);
    new_node.parent = node;
    
    return new_node;
}

point_t generateRandomPoint(int map_dim_x, int map_dim_y){
    point_t point;
    point.x = rand() % map_dim_x;
    point.y = rand() % map_dim_y;
    return point;
}

inline bool doesCollide(rect_t *obstacles, int num_of_obstacles, node_t node){
    // check if vector collides with any of the obstales
    for(int i = 0; i < num_of_obstacles; i++){
        if(node.point.x >= obstacles[i].x1 && node.point.x <= obstacles[i].x2 &&
            node.point.y >= obstacles[i].y1 && node.point.y <= obstacles[i].y2){
            return true;
        }
    }
    return false;
}

bool findNearestNodeToCoordinate(point_t coordinate, node_t *list_of_nodes, int num_of_nodes, node_t **nearest_node){
    // Find the nearest node to the coordinate.
    // returns true if found, 
    // returns false if nearest node is closer than dist_to_grow 
    int min_d2 = pow(map_dim_x + map_dim_y, 2); // Guaranteed to be greater than any distance
    for(int i = 0; i < num_of_nodes; i++) {
        int d2 = squaredDistance(coordinate, list_of_nodes[i].point);
        if(d2 < min_d2){
            min_d2 = d2;
            if(min_d2 < dist_to_grow){
                return false;
            }
            *nearest_node = &list_of_nodes[i];
        }
    }

    return true;
}


int main(){
    srand (time(NULL));
    // Source: https://en.wikipedia.org/wiki/Rapidly-exploring_random_tree#Algorithm
    //
    // Algorithm BuildRRT
    // Input: Initial configuration qinit, number of vertices in RRT K, incremental distance Δq
    // Output: RRT graph G
    //
    // G.init(qinit)
    // for k = 1 to K do
    //     qrand ← RAND_CONF()
    //     qnear ← NEAREST_VERTEX(qrand, G)
    //     qnew ← NEW_CONF(qnear, qrand, Δq)
    //     G.add_vertex(qnew)
    //     G.add_edge(qnear, qnew)
    // return G
    //
    // In the algorithm above, "RAND_CONF" grabs a random configuration qrand in C. 
    // This may be replaced with a function "RAND_FREE_CONF" that uses samples in Cfree, 
    // while rejecting those in Cobs using some collision detection algorithm.
    // 
    // "NEAREST_VERTEX" is a function that runs through all vertices v in graph G, 
    // calculates the distance between qrand and v using some measurement function thereby 
    // returning the nearest vertex.
    // 
    // "NEW_CONF" selects a new configuration qnew by moving an incremental distance Δq 
    // from qnear in the direction of qrand. (According to [4] in holonomic problems, this 
    // should be omitted and qrand used instead of qnew.)

    map_dim_x = 100;
    map_dim_y = 100;
    num_of_obstacles = 1;
    dist_to_grow = 5;
    max_num_of_nodes = 30000;


    // initialize map with obstacles
    rect_t obstacles[num_of_obstacles];
    node_t list_of_nodes[max_num_of_nodes];

    // initialize start and goal
    start.x = (int) map_dim_x/2;
    start.y = (int) map_dim_y/2;
    list_of_nodes[0] = (node_t) {
        .point = start,
        .parent = NULL
    };

    // initialize goal
    goal.x = 10;
    goal.y = 20;

    obstacles[0].x1 = 20;
    obstacles[0].y1 = 20;
    obstacles[0].x2 = 30;
    obstacles[0].y2 = 40;


    int num_nodes_generated = 0;
    for(num_nodes_generated = 1; num_nodes_generated < max_num_of_nodes; num_nodes_generated++){
        node_t *nearest_vertex;
        point_t random_point;
        node_t candidate_node;
        do{
            // genereate a random point
            random_point = generateRandomPoint(map_dim_x, map_dim_y);
            // printf("Random point: (%d, %d)\n", random_point.x, random_point.y);

            // find the nearest vertex
            bool res = findNearestNodeToCoordinate(random_point, list_of_nodes, num_nodes_generated, &nearest_vertex);
            if (!res) {
                // Closest node is too close...just skip this iteration
                continue;
            }
            // printf("Nearest vertex: (%d, %d)\n", nearest_vertex->point.x, nearest_vertex->point.y);
            
            // generate a new node by growing from the nearest vertex
            candidate_node = growFromNode(nearest_vertex, random_point, dist_to_grow);
        // check if node is valid
        } while(doesCollide(obstacles, num_of_obstacles, candidate_node));

        // add node
        list_of_nodes[num_nodes_generated] = candidate_node;
        // printf("Generated node %d at (%d, %d)\n", num_nodes_generated, candidate_node.point.x, candidate_node.point.y);

        // check if point intersects with goal. If so end.
        if(closerThanDistSquared(candidate_node.point, goal, dist_to_grow)){
            break;
        }
    }

    // print out stats
    printf("Number of nodes generated: %d", num_nodes_generated);
}
