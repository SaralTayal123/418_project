#ifndef UTIL_H
#define UTIL_H

#include <cmath>
#include "dataStructs.h"

// Compute eucledian distance between two points.
// USE SQUAREDDISTANCE if possible because sqrt's are very expensive
inline float euclideanDistance(point_t point1, point_t point2){
    return sqrt(pow(point1.x - point2.x, 2) + pow(point1.y - point2.y, 2));
}

inline int squaredDistance(point_t point1, point_t point2){
    return pow(point1.x - point2.x, 2) + pow(point1.y - point2.y, 2);
}

inline bool closerThanDistSquared(point_t point1, point_t point2, float dist_squared){
    return squaredDistance(point1, point2) < dist_squared;
}

inline bool closerThanDist(point_t point1, point_t point2, float dist){
    return euclideanDistance(point1, point2) < dist;
}

#endif