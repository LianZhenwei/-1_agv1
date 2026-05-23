#ifndef RVIZ_H
#define RVIZ_H
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <tf/tf.h>
#include <vector>
#include <map>
#include <string> 
#include "by_global_path_planning/PathPoint.h" 
#include "by_global_path_planning/Path.h" 
class RvizVisualizer
{
public:
    RvizVisualizer(ros::NodeHandle &nh);                                                            
    void publishRoads(const std::map<std::string, std::vector<geometry_msgs::Point>> &road_points); 
    void publishVertices(const std::map<int, geometry_msgs::Point> &node_coords);                   
    void publishGlobalPath(const nav_msgs::Path &global_path);                                      
    void publishVehiclePose(const geometry_msgs::Pose &pose);                                       
    void publishGoalPose(const geometry_msgs::Pose &goal_pose);                                     
    void setVehicleSize(double length, double width)
    {
        vehicle_length_ = length;
        vehicle_width_ = width;
    }
private:
    ros::Publisher pub_roads_;       
    ros::Publisher pub_vertices_;    
    ros::Publisher pub_global_path_; 
    ros::Publisher pub_vehicle_;     
    ros::Publisher pub_goal_;        
    double vehicle_length_ = 1.8;      
    double vehicle_width_ = 0.9;      
    std::string my_frame_id = "odom"; 
    int marker_id_ = 0;
    void resetMarkerId() { marker_id_ = 0; }
    std::vector<geometry_msgs::Point> calculateVehicleRectVertices();
    geometry_msgs::Point rotatePoint(const geometry_msgs::Point &point, double yaw);
};
#endif 
