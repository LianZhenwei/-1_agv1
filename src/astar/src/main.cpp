#include <iostream>
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <opencv2/opencv.hpp>
#include "Astar.h"
#include "OccMapTransform.h"
#include "by_global_path_planning/Path.h"      
#include "by_global_path_planning/PathPoint.h" 
#include <visualization_msgs/MarkerArray.h>    
#include <nav_msgs/Odometry.h>                 
#include "BSpline.h"

using namespace cv;
using namespace std;

struct State 
{
    double x = 0;     
    double y = 0;     
    double yaw = 0;   
    double speed = 0; 
    double yawrate = 0;
};
ros::Subscriber map_sub;
ros::Subscriber startPoint_sub;
ros::Subscriber targetPoint_sub;
ros::Subscriber startPoint_sub_2;
ros::Subscriber targetPoint_sub_2;
ros::Subscriber startPoint_sub_3;  
ros::Subscriber targetPoint_sub_3; 
ros::Subscriber sub_slam;      
ros::Subscriber sub_odom;      
ros::Subscriber sub_end_point; 
ros::Publisher mask_pub;
ros::Publisher path_pub;
ros::Publisher path_pub_2;
ros::Publisher path_pub_3;                
ros::Publisher marker_pub_road_rviz_0922; 
ros::Publisher path_pub_0922;             
nav_msgs::OccupancyGrid OccGridMask;
nav_msgs::Path path;
nav_msgs::Path path_2;
nav_msgs::Path path_3;    
nav_msgs::Path path_0922; 
pathplanning::AstarConfig config;
pathplanning::Astar astar;
OccupancyGridParam OccGridParam;
Point startPoint, targetPoint;
Point startPoint_2, targetPoint_2;
Point startPoint_3, targetPoint_3;       
Point startPoint_0922, targetPoint_0922; 
double l_default_ = -1.0; 
double InflateRadius;
bool map_flag;
bool startpoint_flag;
bool targetpoint_flag;
bool startpoint_flag_2;
bool targetpoint_flag_2;
bool start_flag;
bool start_flag_2;
bool start_flag_0922;       
bool startpoint_flag_0922;  
bool targetpoint_flag_0922; 
bool start_flag_3;          
bool startpoint_flag_3;     
bool targetpoint_flag_3;    
int rate;
State vehicleState; 
bool GPS = -1;  
bool ODOM = -1; 
bool SLAM = -1; 
string my_frame_id = "odom";

void rviz_global_path_planning(ros::Publisher marker_pub, by_global_path_planning::Path globle_path) 
{
    if (globle_path.points.size() < 2) 
    {
        ROS_WARN("Skip publishing path: insufficient points (%ld < 2)", globle_path.points.size());
        return;
    }
    visualization_msgs::Marker points;
    points.header.frame_id = my_frame_id;
    points.header.stamp = ros::Time::now();
    points.ns = "points_and_lines";
    points.action = visualization_msgs::Marker::ADD;
    points.pose.orientation.w = 1.0;
    points.id = 0;
    points.type = visualization_msgs::Marker::LINE_STRIP;
    points.scale.x = 0.05;
    points.scale.y = 0.05;
    points.color.r = 1.0f;
    points.color.a = 1.0;
    for (size_t i = 0; i < globle_path.points.size(); i++)
    {
        geometry_msgs::Point p;
        p.x = globle_path.points[i].x;
        p.y = globle_path.points[i].y;
        p.z = 0;
        points.points.push_back(p);
    }
    marker_pub.publish(points);
}
std::vector<Point> B_spline_optimization(std::vector<Point> all_point) 
{
    std::vector<Point> path_points;
    if (all_point.size() < 4) 
    {
        ROS_WARN("Not enough points for B-spline (min 4 points required)");
        return all_point; 
    }
    int num = all_point.size();
    CPosition *testpt = new CPosition[num];
    for (int i = 0; i < num; i++)
    {
        testpt[i] = CPosition(all_point[i].x, all_point[i].y); 
    }
    int *Intnum = new int[num - 1];
    for (int i = 0; i < num - 1; i++)
    {
        Intnum[i] = 3; 
    }
    int num2 = num;
    CBSpline bspline;
    bspline.ThreeOrderBSplineInterpolatePt(testpt, num2, Intnum); 
    for (int i = 0; i < num2; i++)
    {
        Point p;
        p.x = testpt[i].x;
        p.y = testpt[i].y;
        path_points.push_back(p);
    }
    delete[] Intnum; 
    delete[] testpt; 
    return path_points;
}

void MapCallback(const nav_msgs::OccupancyGrid &msg)
{
    OccGridParam.GetOccupancyGridParam(msg);
    int height = OccGridParam.height;
    int width = OccGridParam.width;
    int OccProb;
    Mat Map(height, width, CV_8UC1);
    for (int i = 0; i < height; i++)
    {
        for (int j = 0; j < width; j++)
        {
            OccProb = msg.data[i * width + j];
            OccProb = (OccProb < 0) ? 100 : OccProb; 
            Map.at<uchar>(height - i - 1, j) = 255 - round(OccProb * 255.0 / 100.0);
        }
    }
    Mat Mask;
    config.InflateRadius = round(InflateRadius / OccGridParam.resolution);
    astar.InitAstar(Map, Mask, config);
    OccGridMask.header.stamp = ros::Time::now();
    OccGridMask.header.frame_id = "map";
    OccGridMask.info = msg.info;
    OccGridMask.data.clear();
    for (int i = 0; i < height; i++)
    {
        for (int j = 0; j < width; j++)
        {
            OccProb = Mask.at<uchar>(height - i - 1, j) * 255;
            OccGridMask.data.push_back(OccProb);
        }
    }
    map_flag = true;
    startpoint_flag = false;
    targetpoint_flag = false;
}

void StartPointCallback(const geometry_msgs::PoseWithCovarianceStamped &msg)
{
    Point2d src_point = Point2d(msg.pose.pose.position.x, msg.pose.pose.position.y);
    OccGridParam.Map2ImageTransform(src_point, startPoint);
    startpoint_flag = true;
    if (map_flag && startpoint_flag && targetpoint_flag)
    {
        start_flag = true;
    }
}
void StartPointCallback_2(const geometry_msgs::PoseWithCovarianceStamped &msg)
{
    Point2d src_point = Point2d(msg.pose.pose.position.x, msg.pose.pose.position.y);
    OccGridParam.Map2ImageTransform(src_point, startPoint_2);
    startpoint_flag_2 = true;
    if (map_flag && startpoint_flag_2 && targetpoint_flag_2)
    {
        start_flag_2 = true;
    }
}
void StartPointCallback_3(const geometry_msgs::PoseWithCovarianceStamped &msg) 
{
    Point2d src_point = Point2d(msg.pose.pose.position.x, msg.pose.pose.position.y);
    OccGridParam.Map2ImageTransform(src_point, startPoint_3);
    startpoint_flag_3 = true;
    if (map_flag && startpoint_flag_3 && targetpoint_flag_3)
    {
        start_flag_3 = true;
    }
}

void TargetPointtCallback(const geometry_msgs::PoseStamped &msg)
{
    Point2d src_point = Point2d(msg.pose.position.x, msg.pose.position.y);
    OccGridParam.Map2ImageTransform(src_point, targetPoint);
    targetpoint_flag = true;
    if (map_flag && startpoint_flag && targetpoint_flag)
    {
        start_flag = true;
    }
}
void TargetPointtCallback_2(const geometry_msgs::PoseStamped &msg)
{
    Point2d src_point = Point2d(msg.pose.position.x, msg.pose.position.y);
    OccGridParam.Map2ImageTransform(src_point, targetPoint_2);
    targetpoint_flag_2 = true;
    if (map_flag && startpoint_flag_2 && targetpoint_flag_2)
    {
        start_flag_2 = true;
    }
}
void TargetPointtCallback_3(const geometry_msgs::PoseStamped &msg) 
{
    Point2d src_point = Point2d(msg.pose.position.x, msg.pose.position.y);
    OccGridParam.Map2ImageTransform(src_point, targetPoint_3);
    targetpoint_flag_3 = true;
    if (map_flag && startpoint_flag_3 && targetpoint_flag_3)
    {
        start_flag_3 = true;
    }
}

void odometryGetCallBack_slam(const nav_msgs::Odometry::ConstPtr odometry_msg) 
{
    if (SLAM)
    {
        if (!map_flag) 
        {
            ROS_WARN_THROTTLE(1.0, "【skipping odometry processing】Map not ready yet, cannot process odometry data"); 
            return;
        }
        ros::Rate loop_rate(10);
        double raw, pitch, theta;
        tf::Quaternion q;
        tf::quaternionMsgToTF(odometry_msg->pose.pose.orientation, q);
        tf::Matrix3x3(q).getRPY(raw, pitch, theta);
        vehicleState.yaw = theta;
        vehicleState.x = odometry_msg->pose.pose.position.x - cos(theta) * (0.9); 
        vehicleState.y = odometry_msg->pose.pose.position.y - sin(theta) * (0.9); 
        loop_rate.sleep();
        geometry_msgs::PoseWithCovarianceStamped start_1;
        start_1.header.stamp = ros::Time::now();
        start_1.header.frame_id = "odom";
        start_1.pose.pose.position.x = vehicleState.x;
        start_1.pose.pose.position.y = vehicleState.y;
        start_1.pose.pose.orientation.w = vehicleState.yaw;
        Point2d src_point = Point2d(start_1.pose.pose.position.x, start_1.pose.pose.position.y);
        OccGridParam.Map2ImageTransform(src_point, startPoint_0922);
        startpoint_flag_0922 = true;
        if (map_flag && startpoint_flag_0922 && targetpoint_flag_0922)
        {
            start_flag_0922 = true;
        }
    }
}
void odometryGetCallBack(const nav_msgs::Odometry::ConstPtr odometry_msg) 
{
    if (ODOM)
    {
        if (!map_flag) 
        {
            ROS_WARN_THROTTLE(1.0, "【skipping odometry processing】Map not ready yet, cannot process odometry data"); 
            return;
        }
        ros::Rate loop_rate(10);
        double raw, pitch, theta;
        tf::Quaternion q;
        tf::quaternionMsgToTF(odometry_msg->pose.pose.orientation, q);
        tf::Matrix3x3(q).getRPY(raw, pitch, theta);
        vehicleState.yaw = theta;
        vehicleState.x = odometry_msg->pose.pose.position.x; 
        vehicleState.y = odometry_msg->pose.pose.position.y; 
        loop_rate.sleep();
        geometry_msgs::PoseWithCovarianceStamped start_1;
        start_1.header.stamp = ros::Time::now();
        start_1.header.frame_id = "odom";
        start_1.pose.pose.position.x = vehicleState.x;
        start_1.pose.pose.position.y = vehicleState.y;
        start_1.pose.pose.orientation.w = vehicleState.yaw;
        Point2d src_point = Point2d(start_1.pose.pose.position.x, start_1.pose.pose.position.y);
        OccGridParam.Map2ImageTransform(src_point, startPoint_0922);
        startpoint_flag_0922 = true;
        if (map_flag && startpoint_flag_0922 && targetpoint_flag_0922)
        {
            start_flag_0922 = true;
        }
    }
}
void end_point_callback(const geometry_msgs::PoseStamped::ConstPtr msg) 
{
    if (!map_flag) 
    {
        ROS_WARN_THROTTLE(1.0, "【skipping end point processing】Map not ready yet, cannot process end point data"); 
        return;
    }
    ros::Rate loop_rate(10);
    Point end_point;                    
    end_point.x = msg->pose.position.x; 
    end_point.y = msg->pose.position.y;
    geometry_msgs::PoseStamped target_2;
    target_2.header.stamp = ros::Time::now();
    target_2.header.frame_id = "world";
    target_2.pose.position.x = end_point.x;
    target_2.pose.position.y = end_point.y;
    target_2.pose.position.z = 0;
    target_2.pose.orientation.x = msg->pose.orientation.x;
    target_2.pose.orientation.y = msg->pose.orientation.y;
    target_2.pose.orientation.z = msg->pose.orientation.z;
    target_2.pose.orientation.w = msg->pose.orientation.w;
    Point2d src_point = Point2d(target_2.pose.position.x, target_2.pose.position.y);
    OccGridParam.Map2ImageTransform(src_point, targetPoint_0922);
    targetpoint_flag_0922 = true;
    if (map_flag && startpoint_flag_0922 && targetpoint_flag_0922)
    {
        start_flag_0922 = true;
    }
    loop_rate.sleep();
}
int main(int argc, char *argv[])
{
    setlocale(LC_CTYPE, "zh_CN.utf8");
    ros::init(argc, argv, "astar");
    ros::NodeHandle nh;
    ros::NodeHandle nh_priv("~");
    ROS_INFO("Start astar node!\n");
    int odom_ = -1; 
    int slam_ = -1;
    int gps_ = -1;
    nh.param<int>("ODOM_", odom_, -1);
    nh.param<int>("SLAM_", slam_, -1);
    nh.param<int>("GPS_", gps_, -1);
    ODOM = odom_;
    SLAM = slam_;
    GPS = gps_;
    ros::param::get("l_default", l_default_); 
    if (SLAM != 1) 
    {
        for (size_t i = 0; i < 20; i++)
        {
            ROS_WARN("Detected SLAM = %d, GPS = %d, ODOM = %d", SLAM, GPS, ODOM);
            ROS_WARN("SLAM != 1 or GPS != 1, please check the parameters in user.yaml!");
        }
        ros::shutdown();    
        exit(EXIT_FAILURE); 
    }
    map_flag = false;
    startpoint_flag = false;
    targetpoint_flag = false;
    start_flag = false;
    startpoint_flag_2 = false;
    targetpoint_flag_2 = false;
    start_flag_2 = false;
    startpoint_flag_0922 = false;  
    targetpoint_flag_0922 = false; 
    start_flag_0922 = false;       
    nh_priv.param<bool>("Euclidean", config.Euclidean, true);
    nh_priv.param<int>("OccupyThresh", config.OccupyThresh, -1);
    nh_priv.param<double>("InflateRadius", InflateRadius, -1);
    nh_priv.param<int>("rate", rate, 10);
    map_sub = nh.subscribe("map", 10, MapCallback);                                          
    startPoint_sub = nh.subscribe("initialpose_1", 10, StartPointCallback);                  
    targetPoint_sub = nh.subscribe("move_base_simple/goal_1", 10, TargetPointtCallback);     
    startPoint_sub_2 = nh.subscribe("initialpose_2", 10, StartPointCallback_2);              
    targetPoint_sub_2 = nh.subscribe("move_base_simple/goal_2", 10, TargetPointtCallback_2); 
    sub_slam = nh.subscribe("/localization", 1, odometryGetCallBack_slam);                   
    sub_odom = nh.subscribe("/odom", 1, odometryGetCallBack);                                
    sub_end_point = nh.subscribe("/move_base_simple/goal", 10, end_point_callback);          
    mask_pub = nh.advertise<nav_msgs::OccupancyGrid>("mask", 1);                               
    path_pub = nh.advertise<nav_msgs::Path>("nav_path", 10);                                   
    path_pub_2 = nh.advertise<nav_msgs::Path>("nav_path_2", 10);                               
    path_pub_0922 = nh.advertise<by_global_path_planning::Path>("/to_control_all_2", 10); // 改     
    marker_pub_road_rviz_0922 = nh.advertise<visualization_msgs::Marker>("road_rviz_0922", 1); 
    startPoint_sub_3 = nh.subscribe("initialpose_3", 10, StartPointCallback_3);              
    targetPoint_sub_3 = nh.subscribe("move_base_simple/goal_3", 10, TargetPointtCallback_3); 
    path_pub_3 = nh.advertise<nav_msgs::Path>("nav_path_3", 10);                             
    ros::Rate loop_rate(rate);
    while (ros::ok())
    {
        if (start_flag)
        {
            double start_time = ros::Time::now().toSec();
            vector<Point> PathList;
            astar.PathPlanning(startPoint, targetPoint, PathList);
            if (!PathList.empty())
            {
                path.header.stamp = ros::Time::now();
                path.header.frame_id = "map";
                path.poses.clear();
                for (int i = 0; i < PathList.size(); i++)
                {
                    Point2d dst_point;
                    OccGridParam.Image2MapTransform(PathList[i], dst_point);
                    geometry_msgs::PoseStamped pose_stamped;
                    pose_stamped.header.stamp = ros::Time::now();
                    pose_stamped.header.frame_id = "map";
                    pose_stamped.pose.position.x = dst_point.x;
                    pose_stamped.pose.position.y = dst_point.y;
                    pose_stamped.pose.position.z = 0;
                    path.poses.push_back(pose_stamped);
                }
                path_pub.publish(path);
                double end_time = ros::Time::now().toSec();
                ROS_INFO("Find a valid path successfully! Use %f s", end_time - start_time);
            }
            else
            {
                ROS_ERROR("Can not find a valid path");
            }
            start_flag = false;
        }
        if (start_flag_2)
        {
            double start_time = ros::Time::now().toSec();
            vector<Point> PathList_2;
            astar.PathPlanning(startPoint_2, targetPoint_2, PathList_2);
            if (!PathList_2.empty())
            {
                path_2.header.stamp = ros::Time::now();
                path_2.header.frame_id = "map";
                path_2.poses.clear();
                for (int i = 0; i < PathList_2.size(); i++)
                {
                    Point2d dst_point_2;
                    OccGridParam.Image2MapTransform(PathList_2[i], dst_point_2);
                    geometry_msgs::PoseStamped pose_stamped_2;
                    pose_stamped_2.header.stamp = ros::Time::now();
                    pose_stamped_2.header.frame_id = "map";
                    pose_stamped_2.pose.position.x = dst_point_2.x;
                    pose_stamped_2.pose.position.y = dst_point_2.y;
                    pose_stamped_2.pose.position.z = 0;
                    path_2.poses.push_back(pose_stamped_2);
                }
                path_pub_2.publish(path_2);
                double end_time = ros::Time::now().toSec();
                ROS_INFO("Find a valid path successfully! Use %f s", end_time - start_time);
            }
            else
            {
                ROS_ERROR("Can not find a valid path");
            }
            start_flag_2 = false;
        }
        if (start_flag_3) 
        {
            double start_time = ros::Time::now().toSec();
            vector<Point> PathList_3;
            astar.PathPlanning(startPoint_3, targetPoint_3, PathList_3);
            if (!PathList_3.empty())
            {
                path_3.header.stamp = ros::Time::now();
                path_3.header.frame_id = "map";
                path_3.poses.clear();
                for (int i = 0; i < PathList_3.size(); i++)
                {
                    Point2d dst_point_3;
                    OccGridParam.Image2MapTransform(PathList_3[i], dst_point_3);
                    geometry_msgs::PoseStamped pose_stamped_3;
                    pose_stamped_3.header.stamp = ros::Time::now();
                    pose_stamped_3.header.frame_id = "map";
                    pose_stamped_3.pose.position.x = dst_point_3.x;
                    pose_stamped_3.pose.position.y = dst_point_3.y;
                    pose_stamped_3.pose.position.z = 0;
                    path_3.poses.push_back(pose_stamped_3);
                }
                path_pub_3.publish(path_3);
                double end_time = ros::Time::now().toSec();
                ROS_INFO("Find a valid path successfully! Use %f s", end_time - start_time);
            }
            else
            {
                ROS_ERROR("Can not find a valid path");
            }
            start_flag_3 = false;
        }
        if (start_flag_0922) 
        {
            double start_time = ros::Time::now().toSec();
            vector<Point> PathList_0922;
            astar.PathPlanning(startPoint_0922, targetPoint_0922, PathList_0922);
            if (!PathList_0922.empty()) 
            {
                vector<Point> smoothed_path;
                if (PathList_0922.size() >= 4)
                {
                    smoothed_path = B_spline_optimization(PathList_0922);
                }
                else
                {
                    ROS_WARN("Not enough points for B-spline smoothing. Using original path.");
                    smoothed_path = PathList_0922; 
                }
                if (!smoothed_path.empty())
                {
                    path_0922.header.stamp = ros::Time::now();
                    path_0922.header.frame_id = "map";
                    path_0922.poses.clear();
                    for (int i = 0; i < smoothed_path.size(); i++)
                    {
                        Point2d dst_point_0922;
                        OccGridParam.Image2MapTransform(smoothed_path[i], dst_point_0922);
                        geometry_msgs::PoseStamped pose_stamped_0922;
                        pose_stamped_0922.header.stamp = ros::Time::now();
                        pose_stamped_0922.header.frame_id = "map";
                        pose_stamped_0922.pose.position.x = dst_point_0922.x;
                        pose_stamped_0922.pose.position.y = dst_point_0922.y;
                        pose_stamped_0922.pose.position.z = 0;
                        path_0922.poses.push_back(pose_stamped_0922);
                    }
                    by_global_path_planning::Path custom_path;
                    for (const auto &pose_stamped : path_0922.poses)
                    {
                        by_global_path_planning::PathPoint pt;
                        pt.x = pose_stamped.pose.position.x;
                        pt.y = pose_stamped.pose.position.y;
                        pt.l = l_default_; 
                        pt.r = 0.0;
                        pt.s = 0.0;
                        pt.heading = 0.0;
                        pt.kappa = 0.0;
                        custom_path.points.push_back(pt);
                    }
                    path_pub_0922.publish(custom_path);
                    rviz_global_path_planning(marker_pub_road_rviz_0922, custom_path);
                    double end_time = ros::Time::now().toSec();
                }
                else
                {
                    ROS_ERROR("-------Can not find a valid path");
                }
            }
            else
            {
                ROS_ERROR("-------A*规划路径为空");
            }
            start_flag_0922 = false;
        }
        if (map_flag)
        {
            mask_pub.publish(OccGridMask);
        }
        loop_rate.sleep();
        ros::spinOnce();
    }
    return 0;
}
