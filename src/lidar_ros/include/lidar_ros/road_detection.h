#ifndef _road_detection_h
#define _road_detection_h
using namespace std;
#define pi 3.14
//ROS功能包下所用的文件
#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include "geometry_msgs/PoseStamped.h"

//c++库
#include <fstream>
#include "vector"
#include <iostream>

int third_road_count = 0;

struct Point
{
    double x;
    double y;
    double l;     //左侧距离
    double r;     //右侧距离
    double s;     //弧长
    double theta; //朝向
};



#endif
