#ifndef DATA_STRUCTS_H
#define DATA_STRUCTS_H

#include <stdint.h>

typedef struct{
    int x1;
    int x2; 
    int y1;
    int y2;
} rect_t;

typedef struct{
    int x;
    int y;
} point_t;

typedef struct node{
    point_t point;
    uint32_t cost;
    struct node *parent;
    // struct node *child;
} node_t;

#endif