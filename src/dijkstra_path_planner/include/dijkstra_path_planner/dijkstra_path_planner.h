#ifndef DIJKSTRA_PATH_PLANNER_H
#define DIJKSTRA_PATH_PLANNER_H
#include <ros/ros.h>
#include <cmath>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <fstream>
#include <sstream>
#include <algorithm>            
#include <cctype>               
#include <boost/filesystem.hpp> 
#include "Graph.h"              
#include "Dijkstra.h"           
#include "rviz.h" 

using namespace std;              
namespace fs = boost::filesystem; 

struct newPoint
{
    double x;
    double y;
    double l;
    double r;
    double s;
    double theta;
};

class PathPlannerNode
{
public:
    PathPlannerNode(ros::NodeHandle &nh); 
    ~PathPlannerNode() = default;         
private:
    ros::NodeHandle nh_;                    
    ros::Subscriber sub_localization_;      
    ros::Subscriber sub_odom_;              
    ros::Subscriber sub_goal_;              
    ros::Publisher pub_global_path_;        
    ros::Publisher pub_nearest_start_node_; 
    ros::Publisher pub_nearest_goal_node_;  
    ros::Timer timer1;                      
    geometry_msgs::Point nearest_start_node_; 
    geometry_msgs::Point nearest_goal_node_;  
    RvizVisualizer rviz_vis_;
    Graphlnk<int, double> graph_;    
    map<int, newPoint> node_coords_; 
    vector<int> node_list_; 
    double max_dist_ = 1e6; 
    int floor = 0;        
    int floor_last = 0;   
    int to_goal = 0;      
    int to_goal_last = 0; 
    map<string, vector<newPoint>> all_road_points_; 
    int start_node_ = -1;          
    int goal_node_ = -1;           
    int second_nearest_node_ = -1; 

    string txt_dir_ = "/home/yt/agv/txt/"; 
    double sample_step_ = 0.1;              
    const string frame_id_ = "map";         
    double l_default_ = -1.0; 

    void initGraphFromTxt();                                                                       
    void initGraph();                                                                              
    void parseTxtFile(const string &file_path);                                                    
    void findNearestNode(const newPoint &pos, int &target_node_id);                                
    void findNearestNode2(const newPoint &pos, int &nearest_node_id, int &second_nearest_node_id); 
    vector<int> getNodePath(int start, int goal);                                   
    void print_node_path(vector<int> node_path, int unchanged0_original1_current2); 
    by_global_path_planning::Path spliceGlobalPath(const vector<int> &node_path);   
    vector<newPoint> readPathPointsFromTxt(const string &file_path, bool show_warn_flag); 
    // geometry_msgs::Point lowPassFilter(const geometry_msgs::Point &current_point);
    // geometry_msgs::Point last_filtered_point_; 

    // bool is_first_point_ = true;                                                  
    // const double filter_alpha_ = 0.3;                                           
    // vector<geometry_msgs::Point> processPathPoints(const vector<geometry_msgs::Point> &input_points); 
    // const double outlier_threshold_factor_ = 2.0;                                                    
    // const int filter_window_size_ = 3;                                                        
    vector<newPoint> processPathPoints(const vector<newPoint> &input_points, double max_dis, int num); 
    double MAX_DIS = 10;
    int NUM = 10; 
    
    void odometryGetCallBack_slam(const nav_msgs::Odometry::ConstPtr &msg); 
    void odometryGetCallBack(const nav_msgs::Odometry::ConstPtr &msg);      
    void goalCb(const geometry_msgs::PoseStamped::ConstPtr &msg);           
    void timer_cb1(const ros::TimerEvent &);                                
};
#endif 
