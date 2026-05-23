#ifndef _headfile_h
#define _headfile_h

#include "ros/ros.h"
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <visualization_msgs/MarkerArray.h>

#include "vector"
#include "PID.h"
#include <fstream>
#include <iostream>
using namespace std;
void range_precote(auto &val, double max, double min)
{
    if (val > max)
    {
        val = max;
    }
    if (val < min)
    {
        val = min;
    }
}
struct Point
{
    double x;
    double y;
    double l;
    double r;
    double s;
    double theta;
};

struct State
{
    double x = 0;     // m
    double y = 0;     // m
    double yaw = 0;   // degree
    double speed = 0; // m/s
    double yawrate = 0;
};

#include "by_global_path_planning/Path.h"
#include "by_global_path_planning/PathPoint.h"
#include "rviz.h"

#endif