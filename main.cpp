#include <stdlib.h>
#include <time.h>
#include "main.h"
#include "dataStructs.h"
#include "util.h"
#include <stdio.h>
#include <string.h>
#include <assert.h>
#include <algorithm>    // std::m
#include <chrono>
#include <omp.h>
using namespace std::chrono;

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
    printf("\t-r <int: 0 = run RRT, 1 = run RRT*> (required)\n");
    printf("\t-d <int: distance to grow: default 5> (required)\n");
}


bool growFromNode(node_t *node, point_t direction, int dist_to_grow, node_t *new_node_return) {
    int x_dist = direction.x - node->point.x;
    int y_dist = direction.y - node->point.y;
    float dist = euclideanDistance(node->point, direction);
    float dist_ratio = dist_to_grow / dist;

    node_t new_node;
    new_node.point.x = node->point.x + (x_dist * dist_ratio);
    new_node.point.y = node->point.y + (y_dist * dist_ratio);
    new_node.cost = node->cost + dist; // Keep track of the cost of the path
    new_node.parent = node;
    if (new_node.point.x < 0 || new_node.point.x >= map_dim_x || new_node.point.y < 0 || new_node.point.y >= map_dim_y)
        return false;
    *new_node_return = new_node;
    return true;
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
        rect_t o = obstacles[i];
        int padding = 3;
        if(node.point.x >= o.x1 - padding && node.point.x <= o.x2 + padding &&
            node.point.y >= o.y1 - padding && node.point.y <= o.y2 + padding){
            return true;
        }
    }
    return false;
}

// Collision checker based off of by: https://stackoverflow.com/questions/5514366/how-to-know-if-a-line-intersects-a-rectangle
inline bool doesOverlapCollide(rect_t *obstacles, int num_of_obstacles, node_t node1, node_t node2){
    for(int i = 0; i < num_of_obstacles; i++){
        int x1_boundry = obstacles[i].x1;
        int x2_boundry = obstacles[i].x2;
        int y1_boundry = obstacles[i].y1;
        int y2_boundry = obstacles[i].y2;

        int padding = 5;
        double rectangleMinX = std::min(x1_boundry, x2_boundry) - padding;
        double rectangleMinY = std::min(y1_boundry, y2_boundry) - padding;
        double rectangleMaxX = std::max(x1_boundry, x2_boundry) + padding;
        double rectangleMaxY = std::max(y1_boundry, y2_boundry) + padding;

        int p1X = node1.point.x;
        int p1Y = node1.point.y;
        int p2X = node2.point.x;
        int p2Y = node2.point.y;
        // Find min and max X for the segment
        double minX = (std::min(p1X, p2X));
        double maxX = (std::max(p1X, p2X));       

        // Find the intersection of the segment's and rectangle's x-projections
        maxX = maxX > rectangleMaxX ? rectangleMaxX : maxX;
        minX = minX < rectangleMinX ? rectangleMinX : minX;

        if (minX > maxX) // Projections don't intersect
        {
            continue;
        }

        // Find corresponding min and max Y for min and max X we found before
        double minY = p1Y;
        double maxY = p2Y;

        double dx = p2X - p1X;

        if (dx != 0) // avoid div0 errors
        {
            double a = (p2Y - p1Y)/dx;
            double b = p1Y - a*p1X;
            minY = a*minX + b;
            maxY = a*maxX + b;
        }

        if (minY > maxY)
        {
            double tmp = maxY;
            maxY = minY;
            minY = tmp;
        }

        // Find the intersection of the segment's and rectangle's y-projections
        maxY = maxY > rectangleMaxY ? rectangleMaxY : maxY;
        minY = minY < rectangleMinY ? rectangleMinY : minY;

        if (minY > maxY) // projections don't intersect
        {
            continue;
        }

        return true; // a prior failed
    }
    return false;
}


// // returns true if it does collide (overlap)
// inline bool doesOverlapCollide(rect_t *obstacles, int num_of_obstacles, node_t node1, node_t node2){
//     // check if node1 to node2 overlaps with any of the obstales
//     for(int i = 0; i < num_of_obstacles; i++){
//         int padding = 3;
//         int x1_boundry = obstacles[i].x1 + padding;
//         int x2_boundry = obstacles[i].x2 + padding;
//         int y1_boundry = obstacles[i].y1 - padding;
//         int y2_boundry = obstacles[i].y2 + padding;

//         assert(x1_boundry <= x2_boundry);
//         assert(y1_boundry <= y2_boundry);

//         int node_1_x = node1.point.x;
//         int node_1_y = node1.point.y;
//         int node_2_x = node2.point.x;
//         int node_2_y = node2.point.y;

//         // return false if node1 and node2 x and y are not in the same boundry
//         if((node_1_x < x1_boundry && node_2_x < x1_boundry) || (node_1_x > x2_boundry && node_2_x > x2_boundry) ||
//            (node_1_y < y1_boundry && node_2_y < y1_boundry) || (node_1_y > y2_boundry && node_2_y > y2_boundry)){
//             continue;
//         }

//         // calculate the y and x of the intersection
//         float slope = ((float)(node_2_y - node_1_y)) / ((float)(node_2_x - node_1_x));
//         float node_y_at_x1 = node_1_y + (x1_boundry - node_1_x) * slope; // y offset + slope * (x offset) 
//         float node_y_at_x2 = node_1_y + (x2_boundry - node_1_x) * slope;
        
//         if ((node_y_at_x1 < std::min(y1_boundry, y2_boundry) || node_y_at_x1 > std::max(y2_boundry, y1_boundry)) &&
//             (node_y_at_x2 < std::min(y1_boundry, y2_boundry) || node_y_at_x2 > std::min(y1_boundry, y2_boundry)))
//             continue;

//         float node_x_at_y1 = node_1_x + (y1_boundry - node_1_y) / slope;
//         float node_x_at_y2 = node_1_x + (y2_boundry - node_1_y) / slope;

//         if ((node_x_at_y1 < std::min(x1_boundry, x2_boundry) || node_x_at_y1 > std::max(x2_boundry, x1_boundry)) &&
//             (node_x_at_y2 < std::min(x1_boundry, x2_boundry) || node_x_at_y2 > std::min(x1_boundry, x2_boundry)))
//             continue;

        
//         // one of the prior conditions failed
//         return true;
//     }

//     return false;

// }

bool findNearestNodeToCoordinate(point_t coordinate, node_t *list_of_nodes, int num_of_nodes, node_t **nearest_node){
    // Find the nearest node to the coordinate.
    // returns true if found, 
    // returns false if nearest node is closer than dist_to_grow     
    int min_d2 = pow(map_dim_x + map_dim_y, 2); // Guaranteed to be greater than any distance
    
    int core_count = omp_get_max_threads();
    int min_d2_array[core_count];
    int min_d2_index_array[core_count];

    for(int i = 0; i < core_count; i++){
        min_d2_array[i] = min_d2;
        min_d2_index_array[i] = -1;
    }
    
    #pragma omp parallel for
    for(int i = 0; i < num_of_nodes; i++) {
        int d2 = squaredDistance(coordinate, list_of_nodes[i].point);
        int tid = omp_get_thread_num();
        // printf("Thread %d: d2 = %d\n", tid, d2);
        if(d2 < min_d2_array[tid]){
            min_d2_array[tid] = d2;
            min_d2_index_array[tid] = i;
        }
    }

    int min_index; 
    // int min_i;
    for(int i = 0; i < core_count; i++){
        if(min_d2_array[i] < min_d2){
            min_d2 = min_d2_array[i];
            min_index = min_d2_index_array[i];
            // min_i = i;
        }
    }

    // printf("min_index: %d\n", min_index);
    // printf("min_index2 : %d\n", min_i);

    *nearest_node = &list_of_nodes[min_index];

    if(min_d2 < dist_to_grow){
        return false;
    }

    return true;
}

inline void run_rrt_star(node_t *node_to_refine, node_t *list_of_nodes, int num_of_nodes_to_search, rect_t *obstacles, int num_of_obstacles){
    static int nodes_improved = 0;
    // Find the nearest node to the coordinate and sets it as its parent.

    int threshold_d2 = dist_to_grow * 2; // just a heuristic, TODO: refine later 
    uint32_t min_d2 = pow(map_dim_x + map_dim_y, 2); // Guaranteed to be greater than any distance
    node_t nearest_node;
    int best_index = -1;
    
    #pragma omp parallel
    {
        #pragma omp for schedule(static, 32)
        for(int i = 0; i < num_of_nodes_to_search; i++) {
            // int num_of_threads = omp_get_num_threads();
            // printf("num_of_threads: %d\n", num_of_threads);

            node_t candidate_node = list_of_nodes[i];
            int d2 = euclideanDistance(node_to_refine->point, candidate_node.point);
            // TODO: Might need to check collision here
            if(d2 < threshold_d2 && d2 > 0){
                bool collide = doesOverlapCollide(obstacles, num_of_obstacles, *node_to_refine, candidate_node);
                if (collide)
                    continue;
                uint32_t cost_with_candidate = candidate_node.cost + d2;
                // valid node, see if the cost is lower
                if (cost_with_candidate < node_to_refine->cost && cost_with_candidate < min_d2) {
                    nearest_node = candidate_node;
                    min_d2 = cost_with_candidate;
                    best_index = i;
                }
            }
        }

        if(min_d2 !=  pow(map_dim_x + map_dim_y, 2)) {

            uint32_t cost_with_candidate = nearest_node.cost + euclideanDistance(node_to_refine->point, nearest_node.point);
            node_to_refine->parent = list_of_nodes + best_index;
            node_to_refine->cost = cost_with_candidate;
                        
            // now that the best node is found, find other nodes that can benifit from this
            
            #pragma omp for schedule(static, 32)
            for(int i = 0; i < num_of_nodes_to_search; i++) {
                node_t *candidate_node = list_of_nodes + i;
                int d2 = euclideanDistance(node_to_refine->point, candidate_node->point);
                // TODO: Might need to check collision here
                bool collide = doesOverlapCollide(obstacles, num_of_obstacles, *node_to_refine, *candidate_node);
                if (collide)
                    continue;
                if(d2 < threshold_d2 && d2 > 0){
                    uint32_t cost_with_optimized_node = d2 + node_to_refine->cost;
                    // valid node, see if the cost is lower
                    if (cost_with_optimized_node < candidate_node->cost) {
                        candidate_node->parent = list_of_nodes + best_index;
                        candidate_node->cost = cost_with_optimized_node;
                    }
                }
            }   
        }
    }

    return;

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

    _argc = argc - 1;
    _argv = argv + 1;

    const char *input_filename = get_option_string("-f", NULL);
    int num_of_threads = get_option_int("-t", 1);
    int rrt_star_flag = get_option_int("-r", 1);
    dist_to_grow = get_option_int("-d", 1);
    
    omp_set_num_threads(num_of_threads);

    int nthreads = omp_get_num_threads();
    printf("Number of threads = %d\n", nthreads);
    printf("2 Number of threads = %d\n", num_of_threads);

    if (input_filename == NULL) {
        printf("Error: You need to specify -f.\n");
        show_help(argv[0]);
        return 1;
    }

    printf("Number of threads: %d\n", num_of_threads);
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
        .cost = 0,
        .parent = NULL
        // .child = NULL
    };

    // random seed based on current time
    // srand (time(NULL));
    srand(0); // TODO: REVERT THIS OR ELSE RANDOMNESS IS NOT RANDOM


    int num_nodes_generated = 0;
    uint32_t best_cost = 0;
    int num_of_winning_nodes = 0;

    uint32_t timers[10] = {0};
    auto start_time = std::chrono::high_resolution_clock::now();
    for(num_nodes_generated = 1; num_nodes_generated < max_num_of_nodes; num_nodes_generated++){

        node_t *nearest_vertex;
        point_t random_point;
        node_t candidate_node;
        
        auto start_1 = high_resolution_clock::now();
        auto end_1 = high_resolution_clock::now();
        auto end_2 = high_resolution_clock::now();
        auto end_3 = high_resolution_clock::now();

        do{
            // genereate a random point

            // timing code
            start_1 = high_resolution_clock::now();

            random_point = generateRandomPoint(map_dim_x, map_dim_y);
            // printf("Random point: (%d, %d)\n", random_point.x, random_point.y);

            // timing code
            end_1 = high_resolution_clock::now();

            // find the nearest vertex
            bool res = findNearestNodeToCoordinate(random_point, list_of_nodes, num_nodes_generated, &nearest_vertex);
            if (!res) {
                // Closest node is too close...just skip this iteration
                continue;
            }

            // timing code
            end_2 = high_resolution_clock::now();
            
            // generate a new node by growing from the nearest vertex
            res = growFromNode(nearest_vertex, random_point, dist_to_grow, &candidate_node);

            // timing code
            end_3 = high_resolution_clock::now();

            if (!res) {
                // Closest node is too close...just skip this iteration
                continue;
            }
        // check if node is valid
        } while(doesCollide(obstacles, num_of_obstacles, candidate_node));


        auto end_4 = high_resolution_clock::now();
        // run RRT* refinement
        if(rrt_star_flag){
            run_rrt_star(&candidate_node, list_of_nodes, num_nodes_generated, obstacles, num_of_obstacles);
        }
        auto end_5 = high_resolution_clock::now();

        assert(candidate_node.point.x >= 0 && candidate_node.point.x < map_dim_x);
        assert(candidate_node.point.y >= 0 && candidate_node.point.y < map_dim_y);

        // add node
        list_of_nodes[num_nodes_generated] = candidate_node;

        // check if the new node is the goal
        if(closerThanDistSquared(candidate_node.point, goal, pow(dist_to_grow, 2))){
            printf("Found goal \n");
            if (num_of_winning_nodes == 0) {
                best_cost = candidate_node.cost;
                final_winning_node = candidate_node;
                num_of_winning_nodes++;
            }
            else{
                if(candidate_node.cost < best_cost){
                    best_cost = candidate_node.cost;
                    final_winning_node = candidate_node;
                }
                num_of_winning_nodes++;
            }
        }

        auto end_6 = high_resolution_clock::now();

        timers[0] += duration_cast<microseconds>(end_1 - start_1).count();
        timers[1] += duration_cast<microseconds>(end_2 - end_1).count();
        timers[2] += duration_cast<microseconds>(end_3 - end_2).count();
        timers[3] += duration_cast<microseconds>(end_4 - end_3).count();
        timers[4] += duration_cast<microseconds>(end_5 - end_4).count();
        timers[5] += duration_cast<microseconds>(end_6 - end_5).count();

    }
    auto end_time = std::chrono::high_resolution_clock::now();

    printf("\n number of winning nodes: %d \n\n", num_of_winning_nodes);

    for(int i = 0; i < 6; i++){
        printf("%ld, ", timers[i]);
    }

    printf("\n\n");

    printf("overall time: %ld\n", duration_cast<microseconds>(end_time - start_time).count());

    /////////////////////////////////////////////////////////////


    // Save the graph/nodes to a file
    char nodes_output[255];
    snprintf(nodes_output, 256, "outputs/%s_%d_nodes.txt", input_basename, num_of_threads);

    FILE *nodes_output_file = fopen(nodes_output, "w");

    fprintf(nodes_output_file, "%d\n", num_nodes_generated);
    for(int node_iter = 0; node_iter < num_nodes_generated; node_iter++) {
        int parent_index = (list_of_nodes[node_iter].parent - list_of_nodes);
        fprintf(nodes_output_file, "%d %d %d\n", list_of_nodes[node_iter].point.x, list_of_nodes[node_iter].point.y, parent_index);
    }
    printf("Done writing nodes list to file: %s\n", nodes_output);

    // print out stats
    printf("Number of nodes generated: %d \n", num_nodes_generated);

    if (num_of_winning_nodes == 0) {
        printf("Max number of nodes reached and no path found.\n");
        printf("Exiting gracefully without saving path. Please increase max num nodes \n");
        return -1;
    }

    
    /////////////////////////////////////////////////////////////
    

    // traverse the path backwards and print out the points
    node_t current_node = final_winning_node;
    int path_length = 0;
    while(current_node.point.x != start.x || current_node.point.y != start.y){
        int parent_index = (current_node.parent - list_of_nodes);// / sizeof(node_t);
        // printf("%d %d %d\n", current_node.point.x, current_node.point.y, parent_index);
        current_node = list_of_nodes[parent_index];
        path_length++;
    }
    printf("Path length: %d\n", path_length);
    printf("Path Cost: %d\n", final_winning_node.cost);


    // Save the path to a file
    char path_output[255];
    snprintf(path_output, 256, "outputs/%s_%d_path.txt", input_basename, num_of_threads);
    FILE *path_output_file = fopen(path_output, "w");

    fprintf(path_output_file, "%d\n", path_length);
    fprintf(path_output_file, "%d\n", final_winning_node.cost);

    // traverse again to print
    current_node = final_winning_node;
    int parent_index;
    while(current_node.point.x != start.x || current_node.point.y != start.y){
        fprintf(path_output_file, "%d %d %d\n", current_node.point.x, current_node.point.y, parent_index);
        parent_index = (current_node.parent - list_of_nodes);// / sizeof(node_t);
        current_node = list_of_nodes[parent_index];
    }
    fprintf(path_output_file, "%d %d %d\n", current_node.point.x, current_node.point.y, parent_index);

    printf("Writing nodes list to file: %s\n", path_output);
    
}
