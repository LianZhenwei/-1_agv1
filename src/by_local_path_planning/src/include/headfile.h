#ifndef _HEADFILE_H_
#define _HEADFILE_H_

#include "std_msgs/Float64MultiArray.h"
#include "apriltag_ros/AprilTagDetectionArray.h"
#include "ros/ros.h"
#include<iostream>
#include<limits>
#include<sys/time.h>
#include "tf/tf.h"
#include <nav_msgs/Odometry.h>
#include "tf/tf.h"
#include <visualization_msgs/MarkerArray.h>
#include <fstream>

#include "by_global_path_planning/Path.h"
#include "by_global_path_planning/PathPoint.h"
#include "Box2d.h"
#include "rviz.h"
#include "PID.h"

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


#endif