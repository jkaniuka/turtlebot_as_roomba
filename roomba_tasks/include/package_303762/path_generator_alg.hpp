#ifndef PATH_GENERATOR_ALG_HPP
#define PATH_GENERATOR_ALG_HPP


#include <ros/ros.h>
#include <math.h>
#include <stdio.h>
#include <vector>
#include <algorithm>
#include <iostream>

using namespace std;

// struct describing coordinates of a point in the path
typedef struct point_t {
    float x;
    float y;
    float z;
    float w;
} point_t;

// simple test function that prints "Hello"
// can be used to test if header file is included correctly
void sayHello();

// computes a discrete path of points
vector<point_t> generate_discrete_path(float x_start,float y_start,float x_end,float y_end);
#endif