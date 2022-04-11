#include <stdlib.h>
#include <time.h>
#include "main.h"
#include "dataStructs.h"
#include "util.h"
#include <stdio.h>
#include <string.h>

point_t start, goal;
int max_num_of_nodes, num_of_obstacles;
int map_dim_x, map_dim_y;
int dist_to_grow;
node_t final_winning_node;

//copied from assignment 3
static int _argc;
static const char **_argv;

const char *get_option_string(const char *option_name, const char *default_value) {
    for (int i = _argc - 2; i >= 0; i -= 2)
        if (strcmp(_argv[i], option_name) == 0)
            return _argv[i + 1];
    return default_value;
}

int get_option_int(const char *option_name, int default_value) {
    for (int i = _argc - 2; i >= 0; i -= 2)
        if (strcmp(_argv[i], option_name) == 0)
            return atoi(_argv[i + 1]);
    return default_value;
}

float get_option_float(const char *option_name, float default_value) {
    for (int i = _argc - 2; i >= 0; i -= 2)
        if (strcmp(_argv[i], option_name) == 0)
            return (float)atof(_argv[i + 1]);
    return default_value;
}

static void show_help(const char *program_path) {
    printf("Usage: %s OPTIONS\n", program_path);
    printf("\n");
    printf("OPTIONS:\n");
    printf("\t-f <input_filename> (required)\n");
    printf("\t-n <num_of_threads> (required)\n");
    printf("\t-p <SA_prob>\n");
    printf("\t-i <SA_iters>\n");
}




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


int main(int argc, const char *argv[]) {    
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

    int error = 0;

    _argc = argc - 1;
    _argv = argv + 1;

    const char *input_filename = get_option_string("-f", NULL);
    int num_of_threads = get_option_int("-n", 1);
    // double SA_prob = get_option_float("-p", 0.1f);
    // int SA_iters = get_option_int("-i", 5);

    if (input_filename == NULL) {
        printf("Error: You need to specify -f.\n");
        error = 1;
    }

    if (error) {
        show_help(argv[0]);
        return 1;
    }

    printf("Number of threads: %d\n", num_of_threads);
    // printf("Probability parameter for simulated annealing: %lf.\n", SA_prob);
    // printf("Number of simulated annealing iterations: %d\n", SA_iters);
    printf("Input file: %s\n", input_filename);


    // Parse input filepath for basename
    char *input_basename = strdup(input_filename);
    char *basename_ptr = strrchr(input_basename, '/');
    if (basename_ptr != NULL) {
        basename_ptr++;
        input_basename = basename_ptr;
    }
    char *ext = strrchr(input_basename, '.');
    if (ext != NULL)
        *ext = '\0';


    FILE *input = fopen(input_filename, "r");

    if (!input) {
        printf("Unable to open file: %s.\n", input_filename);
        return 1;
    }

    fscanf(input, "%d %d\n", &map_dim_x, &map_dim_y);
    fscanf(input, "%d\n", &max_num_of_nodes);
    fscanf(input, "%d %d\n", &start.x, &start.y);
    fscanf(input, "%d %d\n", &goal.x, &goal.y);
    fscanf(input, "%d\n", &num_of_obstacles);
    printf("Dimensions: %d x %d\n", map_dim_x, map_dim_y);
    printf("Max number of nodes: %d\n", max_num_of_nodes);
    printf("Start: (%d, %d)\n", start.x, start.y);
    printf("Goal: (%d, %d)\n", goal.x, goal.y);
    printf("Number of obstacles: %d\n", num_of_obstacles);


    // initialize map with obstacles
    rect_t obstacles[num_of_obstacles];
    for(int obs_incr = 0; obs_incr < num_of_obstacles; obs_incr++){
        fscanf(input, "%d %d %d %d\n", &obstacles[obs_incr].x1, &obstacles[obs_incr].y1, &obstacles[obs_incr].x2, &obstacles[obs_incr].y2);
    }

    // initialize start and goal
    node_t list_of_nodes[max_num_of_nodes];
    list_of_nodes[0] = (node_t) {
        .point = start,
        .parent = NULL
    };

    // random seed based on current time
    srand (time(NULL));
    // TODO: Move to an argument
    dist_to_grow = 5;


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
            final_winning_node = candidate_node;
            printf("Winning Goal node: (%d, %d)\n", candidate_node.point.x, candidate_node.point.y);
            break;
        }
    }

    // print out stats
    printf("Number of nodes generated: %d \n", num_nodes_generated);

    if (num_nodes_generated == max_num_of_nodes) {
        printf("Max number of nodes reached.\n");
        printf("Exiting gracefully without saving. Please increase max num nodes \n");
        return -1;
    }


    /////////////////////////////////////////////////////////////


    // Save the graph/nodes to a file
    char nodes_output[255];
    snprintf(nodes_output, 256, "outputs/%s_%d_nodes.txt", input_basename, num_of_threads);

    FILE *nodes_output_file = fopen(nodes_output, "w");

    fprintf(nodes_output_file, "%d\n", num_nodes_generated);
    for(int node_iter = 0; node_iter < num_nodes_generated; node_iter++) {
        int parent_index = (list_of_nodes[node_iter].parent - list_of_nodes) / sizeof(node_t);
        fprintf(nodes_output_file, "%d %d %d\n", list_of_nodes[node_iter].point.x, list_of_nodes[node_iter].point.y, parent_index);
    }
    printf("Done writing nodes list to file: %s\n", nodes_output);

    
    /////////////////////////////////////////////////////////////
    

    // traverse the path backwards and print out the points
    node_t current_node = final_winning_node;
    int path_length = 0;
    while(current_node.point.x != start.x || current_node.point.y != start.y){
        int parent_index = (current_node.parent - list_of_nodes) / sizeof(node_t);
        printf("%d %d %d\n", current_node.point.x, current_node.point.y, parent_index);
        current_node = list_of_nodes[parent_index];
        path_length++;
    }
    printf("Path length: %d\n", path_length);


    // Save the path to a file
    char path_output[255];
    snprintf(path_output, 256, "outputs/%s_%d_path.txt", input_basename, num_of_threads);
    FILE *path_output_file = fopen(path_output, "w");

    fprintf(path_output_file, "%d\n", path_length);

    // traverse again to print
    current_node = final_winning_node;
    int parent_index;
    while(current_node.point.x != start.x || current_node.point.y != start.y){
        fprintf(path_output_file, "%d %d %d\n", current_node.point.x, current_node.point.y, parent_index);
        parent_index = (current_node.parent - list_of_nodes) / sizeof(node_t);
        current_node = list_of_nodes[parent_index];
    }
    fprintf(path_output_file, "%d %d %d\n", current_node.point.x, current_node.point.y, parent_index);

    printf("Writing nodes list to file: %s\n", path_output);
    
}
