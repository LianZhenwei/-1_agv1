#include "include/headfile.h"
#include <path_planning_msgs/PathPoint.h>
#include <path_planning_msgs/Path.h>
#include <std_msgs/Bool.h>
#include "geometry_msgs/PoseStamped.h"
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <cmath> 

bool GPS = -1;
bool ODOM = -1;
bool SLAM = -1;

class map_control
{
public:
    double l_default_ = -1.0; 
    int task = 0;
    bool dis_start = 0; 
    bool dis_end = 0;   
    bool end_1 = 0;     
    bool end_pub = 0;
    bool start_pub = 0;
    bool end_pub_finish = 0;
    bool start_pub_finish = 0;
    int floor = 0;        
    int floor_last = 0;   
    int to_goal = 0;      
    int to_goal_last = 0; 
    bool first_flag = 0;
    bool first_flag_1 = 0;
    bool first_flag_2 = 0;
    bool pub_flag = 0;
    Point end_point_2;
    void get_control_callback(const by_global_path_planning::Path msg); 
    void node_start(int argc, char *argv[]);
    vector<Point> track_road;
    vector<Point> track_road_1;
    vector<Point> track_road_2;
    std::vector<Point> track_road_1_new;
    std::vector<Point> track_road_2_new;
    std::vector<Point> track_road_1_b;
    std::vector<Point> track_road_2_b;
    void astar_start_callback(const nav_msgs::Path msg);
    void astar_start_callback_2(const nav_msgs::Path msg);
    State vehicleState;
    bool start_end = 0; 
    void odometryGetCallBack_slam(const nav_msgs::Odometry::ConstPtr odometry_msg);
    void odometryGetCallBack(const nav_msgs::Odometry::ConstPtr odometry_msg);
    void end_point_callback(const geometry_msgs::PoseStamped::ConstPtr msg);
    ros::Publisher pub_astar_start_1;
    ros::Publisher pub_astar_end_1;
    ros::Publisher pub_astar_start_2;
    ros::Publisher pub_astar_end_2;
    ros::Publisher pub_to_control_all;
    ros::Publisher marker_pub_road_rviz_1;
    ros::Publisher marker_pub_road_rviz_2;
    std::vector<Point> low_pass_filter_road_2(vector<Point> &roads, int num, float max_dis);
    ros::Subscriber sub_nearest_start_node_;  
    ros::Subscriber sub_nearest_goal_node_;  
    geometry_msgs::Point nearest_start_node_; 
    geometry_msgs::Point nearest_goal_node_; 
    bool received_nearest_start_ = false;    
    bool received_nearest_goal_ = false;    
    bool start_only_a_single_astar_path = false; 
    bool first = true;                          
    void nearestStartNodeCallback(const geometry_msgs::PointStamped::ConstPtr &msg);
    void nearestGoalNodeCallback(const geometry_msgs::PointStamped::ConstPtr &msg);
    ros::Subscriber sub_astar_path_3;   
    ros::Publisher pub_astar_start_3; 
    ros::Publisher pub_astar_end_3;     
    ros::Publisher marker_pub_road_rviz_3;
    void astarPath3Callback(const nav_msgs::Path msg);
    map_control(); 
    ////////////////////////////////////////////////////////////////////////////////////
};

map_control::map_control()
{
    nearest_start_node_.x = std::nan(""); 
    nearest_start_node_.y = std::nan(""); 
    nearest_start_node_.z = 0.0;  
    nearest_goal_node_.x = std::nan("");
    nearest_goal_node_.y = std::nan("");
    nearest_goal_node_.z = 0.0;
}
void map_control::nearestStartNodeCallback(const geometry_msgs::PointStamped::ConstPtr &msg)
{
    nearest_start_node_ = msg->point;
    received_nearest_start_ = true;
}
void map_control::nearestGoalNodeCallback(const geometry_msgs::PointStamped::ConstPtr &msg)
{
    nearest_goal_node_ = msg->point;
    received_nearest_goal_ = true;
}
void map_control::astarPath3Callback(const nav_msgs::Path msg)
{
    vector<Point> new_path;
    if (msg.poses.size() <= 1)
    {
        ROS_WARN("astarPath3Callback: 路径节点数不足，需要至少2个节点，当前: %zu", msg.poses.size());
        return;
    }
    for (size_t i = 1; i < msg.poses.size() - 1; i++)
    {
        Point p;
        p.x = msg.poses[i].pose.position.x;
        p.y = msg.poses[i].pose.position.y;
        p.l = l_default_;
        new_path.push_back(p);
    }
    if (new_path.empty())
    {
        ROS_WARN("astarPath3Callback: 过滤后路径为空");
        return;
    }
    vector<Point> filtered_path = low_pass_filter_road_2(new_path, 25, 10);
    if (filtered_path.empty())
    {
        ROS_WARN("astarPath3Callback: 滤波后路径为空，不发布");
        start_only_a_single_astar_path = false; 
        return;
    }
    if (filtered_path.empty() || filtered_path.size() < 2)
    {
        ROS_WARN("路径无效，点数: %zu", filtered_path.size());
        start_only_a_single_astar_path = false;
        return;
    }
    for (const auto &p : filtered_path)
    {
        if (std::isnan(p.x) || std::isnan(p.y))
        {
            ROS_WARN("发现无效坐标，不发布路径");
            start_only_a_single_astar_path = false;
            return;
        }
    }
    rviz_road(marker_pub_road_rviz_3, filtered_path, 1, 0, 0); 
    by_global_path_planning::Path globle_path;
    for (const auto &p : filtered_path)
    {
        by_global_path_planning::PathPoint pt;
        pt.x = p.x;
        pt.y = p.y;
        pt.l = p.l;
        pt.r = p.r;
        globle_path.points.push_back(pt);
    }
    pub_to_control_all.publish(globle_path);
    start_only_a_single_astar_path = false; 
}

void map_control::get_control_callback(const by_global_path_planning::Path msg)
{
    track_road.clear();
    start_end = 0;
    first_flag = 0;
    pub_flag = 0;
    for (int i = 0; i < (msg.points.size() - 1); i++)
    {
        Point p;
        p.x = msg.points[i].x;
        p.y = msg.points[i].y;
        p.theta = msg.points[i].heading;
        p.l = msg.points[i].l;
        p.r = msg.points[i].r;
        track_road.push_back(p);
    }
    first_flag = 1;
    start_end = 1;
    pub_flag = 1; 
}

void map_control::odometryGetCallBack_slam(const nav_msgs::Odometry::ConstPtr odometry_msg)
{
    if (SLAM)
    {
        double raw, pitch, theta;
        tf::Quaternion q;
        tf::quaternionMsgToTF(odometry_msg->pose.pose.orientation, q);
        tf::Matrix3x3(q).getRPY(raw, pitch, theta);
        this->vehicleState.yaw = theta;
        this->vehicleState.x = odometry_msg->pose.pose.position.x - cos(theta) * (0.9); 
        this->vehicleState.y = odometry_msg->pose.pose.position.y - sin(theta) * (0.9); 
        if (start_end && received_nearest_start_ && received_nearest_goal_)
        {
            start_end = 0; 
            double node_dist = sqrt(pow(nearest_start_node_.x - nearest_goal_node_.x, 2) + pow(nearest_start_node_.y - nearest_goal_node_.y, 2));
            if (node_dist < 0.1) 
            {
                start_only_a_single_astar_path = true; 
                pub_flag = 0;                          
                geometry_msgs::PoseWithCovarianceStamped start_3;
                start_3.header.stamp = ros::Time::now();
                start_3.header.frame_id = "odom";
                start_3.pose.pose.position.x = vehicleState.x;
                start_3.pose.pose.position.y = vehicleState.y;
                start_3.pose.pose.orientation.w = vehicleState.yaw;
                pub_astar_start_3.publish(start_3);
                geometry_msgs::PoseStamped goal_3;
                goal_3.header.stamp = ros::Time::now();
                goal_3.header.frame_id = "world";
                goal_3.pose.position.x = end_point_2.x;
                goal_3.pose.position.y = end_point_2.y;
                goal_3.pose.orientation.w = end_point_2.theta;
                pub_astar_end_3.publish(goal_3);
            }
            if (start_only_a_single_astar_path == true) 
            {
                return;
            }
            //
            double dist_to_start = sqrt(pow(vehicleState.x - nearest_start_node_.x, 2) + pow(vehicleState.y - nearest_start_node_.y, 2));
            if (dist_to_start > 4.0 && start_only_a_single_astar_path == false) 
            {
                dis_start = 1;
                start_pub = 1;
                geometry_msgs::PoseWithCovarianceStamped start_1;
                start_1.header.stamp = ros::Time::now();
                start_1.header.frame_id = "odom";
                start_1.pose.pose.position.x = this->vehicleState.x;
                start_1.pose.pose.position.y = this->vehicleState.y;
                start_1.pose.pose.orientation.w = this->vehicleState.yaw;
                pub_astar_start_1.publish(start_1);
                geometry_msgs::PoseStamped target_1;
                target_1.header.stamp = ros::Time::now();
                target_1.header.frame_id = "world";
                target_1.pose.position.x = nearest_start_node_.x;
                target_1.pose.position.y = nearest_start_node_.y;
                target_1.pose.position.z = 0;
                target_1.pose.orientation.x = 0;
                target_1.pose.orientation.y = 0;
                target_1.pose.orientation.z = 0;
                target_1.pose.orientation.w = 0;
                pub_astar_end_1.publish(target_1);
            }
        }
        if (first_flag_1)
        {
            first_flag_1 = 0;
            for (int i = track_road_1_new.size(); i > 0; i--)
            {
                Point p;
                p.x = track_road_1_new[i - 1].x;
                p.y = track_road_1_new[i - 1].y;
                p.theta = track_road_1_new[i - 1].theta; 
                p.l = track_road_1_new[i - 1].l;         
                p.r = track_road_1_new[i - 1].r;
                track_road.insert(track_road.begin(), p);
            }
            start_pub_finish = 1;
        }
        if (first_flag_2)
        {
            first_flag_2 = 0;
            for (int i = 0; i < track_road_2_new.size(); i++)
            {
                Point p;
                p.x = track_road_2_new[i].x;
                p.y = track_road_2_new[i].y;
                p.theta = track_road_2_new[i].theta; 
                p.l = track_road_2_new[i].l;         
                p.r = track_road_2_new[i].r; 
                track_road.push_back(p);
            }
            end_pub_finish = 1;
        }
        if (pub_flag && (start_only_a_single_astar_path == false)) 
        {
            if (!start_pub && !end_pub)
            {
                pub_flag = 0;
                by_global_path_planning::Path globle_path_initial;
                for (size_t j = 0; j < track_road.size(); j++)
                {
                    by_global_path_planning::PathPoint p;
                    p.x = track_road[j].x;
                    p.y = track_road[j].y;
                    p.l = track_road[j].l;
                    p.r = track_road[j].r;
                    globle_path_initial.points.push_back(p);
                }
                by_global_path_planning::PathPoint p;
                p.x = end_point_2.x;
                p.y = end_point_2.y;
                p.heading = end_point_2.theta;
                if (!globle_path_initial.points.empty())
                {
                    p.l = globle_path_initial.points.back().l; 
                    p.r = globle_path_initial.points.back().r; 
                }
                else
                {
                    p.l = l_default_; 
                    p.r = 0.0;
                }
                globle_path_initial.points.push_back(p);
                pub_to_control_all.publish(globle_path_initial);
            }
            else if (start_pub && !end_pub)
            {
                if (start_pub_finish)
                {
                    pub_flag = 0;
                    start_pub = 0;
                    start_pub_finish = 0;
                    by_global_path_planning::Path globle_path_initial;
                    for (size_t j = 0; j < track_road.size(); j++)
                    {
                        by_global_path_planning::PathPoint p;
                        p.x = track_road[j].x;
                        p.y = track_road[j].y;
                        p.l = track_road[j].l;
                        p.r = track_road[j].r;
                        globle_path_initial.points.push_back(p);
                    }
                    by_global_path_planning::PathPoint p;
                    p.x = end_point_2.x;
                    p.y = end_point_2.y;
                    p.heading = end_point_2.theta;
                    if (!globle_path_initial.points.empty())
                    {
                        p.l = globle_path_initial.points.back().l; 
                        p.r = globle_path_initial.points.back().r; 
                    }
                    else
                    {
                        p.l = l_default_; 
                        p.r = 0.0;
                    }
                    globle_path_initial.points.push_back(p);
                    pub_to_control_all.publish(globle_path_initial);
                }
            }
            else if (!start_pub && end_pub)
            {
                if (end_pub_finish)
                {
                    pub_flag = 0;
                    end_pub = 0;
                    end_pub_finish = 0;
                    by_global_path_planning::Path globle_path_initial;
                    for (size_t j = 0; j < track_road.size(); j++)
                    {
                        by_global_path_planning::PathPoint p;
                        p.x = track_road[j].x;
                        p.y = track_road[j].y;
                        p.l = track_road[j].l;
                        p.r = track_road[j].r;
                        globle_path_initial.points.push_back(p);
                    }
                    by_global_path_planning::PathPoint p;
                    p.x = end_point_2.x;
                    p.y = end_point_2.y;
                    p.heading = end_point_2.theta;
                    if (!globle_path_initial.points.empty())
                    {
                        p.l = globle_path_initial.points.back().l; 
                        p.r = globle_path_initial.points.back().r; 
                    }
                    else
                    {
                        p.l = l_default_; 
                        p.r = 0.0;
                    }
                    globle_path_initial.points.push_back(p);
                    pub_to_control_all.publish(globle_path_initial);
                }
            }
            else if (start_pub && end_pub)
            {
                if (end_pub_finish && start_pub_finish)
                {
                    pub_flag = 0;
                    end_pub = 0;
                    end_pub_finish = 0;
                    start_pub = 0;
                    start_pub_finish = 0;
                    by_global_path_planning::Path globle_path_initial;
                    for (size_t j = 0; j < track_road.size(); j++)
                    {
                        by_global_path_planning::PathPoint p;
                        p.x = track_road[j].x;
                        p.y = track_road[j].y;
                        p.l = track_road[j].l;
                        p.r = track_road[j].r;
                        globle_path_initial.points.push_back(p);
                    }
                    by_global_path_planning::PathPoint p;
                    p.x = end_point_2.x;
                    p.y = end_point_2.y;
                    p.heading = end_point_2.theta;
                    if (!globle_path_initial.points.empty())
                    {
                        p.l = globle_path_initial.points.back().l; 
                        p.r = globle_path_initial.points.back().r; 
                    }
                    else
                    {
                        p.l = l_default_; 
                        p.r = 0.0;
                    }
                    globle_path_initial.points.push_back(p);
                    pub_to_control_all.publish(globle_path_initial);
                }
            }
            //
        }
    }
}
void map_control::odometryGetCallBack(const nav_msgs::Odometry::ConstPtr odometry_msg)
{
    if (ODOM)
    {
        double raw, pitch, theta;
        tf::Quaternion q;
        tf::quaternionMsgToTF(odometry_msg->pose.pose.orientation, q);
        tf::Matrix3x3(q).getRPY(raw, pitch, theta);
        this->vehicleState.yaw = theta;
        this->vehicleState.x = odometry_msg->pose.pose.position.x; 
        this->vehicleState.y = odometry_msg->pose.pose.position.y; 
        if (start_end && received_nearest_start_ && received_nearest_goal_)
        {
            start_end = 0; 
            double node_dist = sqrt(pow(nearest_start_node_.x - nearest_goal_node_.x, 2) + pow(nearest_start_node_.y - nearest_goal_node_.y, 2));
            if (node_dist < 0.1) 
            {
                start_only_a_single_astar_path = true; 
                pub_flag = 0;
                geometry_msgs::PoseWithCovarianceStamped start_3;
                start_3.header.stamp = ros::Time::now();
                start_3.header.frame_id = "odom";
                start_3.pose.pose.position.x = vehicleState.x;
                start_3.pose.pose.position.y = vehicleState.y;
                start_3.pose.pose.orientation.w = vehicleState.yaw;
                pub_astar_start_3.publish(start_3);
                geometry_msgs::PoseStamped goal_3;
                goal_3.header.stamp = ros::Time::now();
                goal_3.header.frame_id = "world";
                goal_3.pose.position.x = end_point_2.x;
                goal_3.pose.position.y = end_point_2.y;
                goal_3.pose.orientation.w = end_point_2.theta;
                pub_astar_end_3.publish(goal_3);
            }
            if (start_only_a_single_astar_path == true) 
            {
                return;
            }
            //
            double dist_to_start = sqrt(pow(vehicleState.x - nearest_start_node_.x, 2) + pow(vehicleState.y - nearest_start_node_.y, 2));
            if (dist_to_start > 4.0 && start_only_a_single_astar_path == false) 
            {
                dis_start = 1;
                start_pub = 1;
                geometry_msgs::PoseWithCovarianceStamped start_1;
                start_1.header.stamp = ros::Time::now();
                start_1.header.frame_id = "odom";
                start_1.pose.pose.position.x = this->vehicleState.x;
                start_1.pose.pose.position.y = this->vehicleState.y;
                start_1.pose.pose.orientation.w = this->vehicleState.yaw;
                pub_astar_start_1.publish(start_1);
                geometry_msgs::PoseStamped target_1;
                target_1.header.stamp = ros::Time::now();
                target_1.header.frame_id = "world";
                target_1.pose.position.x = nearest_start_node_.x;
                target_1.pose.position.y = nearest_start_node_.y;
                target_1.pose.position.z = 0;
                target_1.pose.orientation.x = 0;
                target_1.pose.orientation.y = 0;
                target_1.pose.orientation.z = 0;
                target_1.pose.orientation.w = 0;
                pub_astar_end_1.publish(target_1);
            }
        }
        if (first_flag_1)
        {
            first_flag_1 = 0;
            for (int i = track_road_1_new.size(); i > 0; i--)
            {
                Point p;
                p.x = track_road_1_new[i - 1].x;
                p.y = track_road_1_new[i - 1].y;
                p.theta = track_road_1_new[i - 1].theta; 
                p.l = track_road_1_new[i - 1].l;         
                p.r = track_road_1_new[i - 1].r; 
                track_road.insert(track_road.begin(), p);
            }
            start_pub_finish = 1;
        }
        if (first_flag_2)
        {
            first_flag_2 = 0;
            for (int i = 0; i < track_road_2_new.size(); i++)
            {
                Point p;
                p.x = track_road_2_new[i].x;
                p.y = track_road_2_new[i].y;
                p.theta = track_road_2_new[i].theta; 
                p.l = track_road_2_new[i].l;         
                p.r = track_road_2_new[i].r;
                track_road.push_back(p);
            }
            end_pub_finish = 1;
        }
        if (pub_flag && (start_only_a_single_astar_path == false))
        {
            if (!start_pub && !end_pub)
            {
                pub_flag = 0;
                by_global_path_planning::Path globle_path_initial;
                for (size_t j = 0; j < track_road.size(); j++)
                {
                    by_global_path_planning::PathPoint p;
                    p.x = track_road[j].x;
                    p.y = track_road[j].y;
                    p.l = track_road[j].l;
                    p.r = track_road[j].r;
                    globle_path_initial.points.push_back(p);
                }
                by_global_path_planning::PathPoint p;
                p.x = end_point_2.x;
                p.y = end_point_2.y;
                p.heading = end_point_2.theta;
                if (!globle_path_initial.points.empty())
                {
                    p.l = globle_path_initial.points.back().l; 
                    p.r = globle_path_initial.points.back().r; 
                }
                else
                {
                    p.l = l_default_; 
                    p.r = 0.0;
                }
                globle_path_initial.points.push_back(p);
                pub_to_control_all.publish(globle_path_initial);
            }
            else if (start_pub && !end_pub)
            {
                if (start_pub_finish)
                {
                    pub_flag = 0;
                    start_pub = 0;
                    start_pub_finish = 0;
                    by_global_path_planning::Path globle_path_initial;
                    for (size_t j = 0; j < track_road.size(); j++)
                    {
                        by_global_path_planning::PathPoint p;
                        p.x = track_road[j].x;
                        p.y = track_road[j].y;
                        p.l = track_road[j].l;
                        p.r = track_road[j].r;
                        globle_path_initial.points.push_back(p);
                    }
                    by_global_path_planning::PathPoint p;
                    p.x = end_point_2.x;
                    p.y = end_point_2.y;
                    p.heading = end_point_2.theta;
                    if (!globle_path_initial.points.empty())
                    {
                        p.l = globle_path_initial.points.back().l; 
                        p.r = globle_path_initial.points.back().r; 
                    }
                    else
                    {
                        p.l = l_default_; 
                        p.r = 0.0;
                    }
                    globle_path_initial.points.push_back(p);
                    pub_to_control_all.publish(globle_path_initial);
                }
            }
            else if (!start_pub && end_pub)
            {
                if (end_pub_finish)
                {
                    pub_flag = 0;
                    end_pub = 0;
                    end_pub_finish = 0;
                    by_global_path_planning::Path globle_path_initial;
                    for (size_t j = 0; j < track_road.size(); j++)
                    {
                        by_global_path_planning::PathPoint p;
                        p.x = track_road[j].x;
                        p.y = track_road[j].y;
                        p.l = track_road[j].l;
                        p.r = track_road[j].r;
                        globle_path_initial.points.push_back(p);
                    }
                    by_global_path_planning::PathPoint p;
                    p.x = end_point_2.x;
                    p.y = end_point_2.y;
                    p.heading = end_point_2.theta;
                    if (!globle_path_initial.points.empty())
                    {
                        p.l = globle_path_initial.points.back().l; 
                        p.r = globle_path_initial.points.back().r; 
                    }
                    else
                    {
                        p.l = l_default_; 
                        p.r = 0.0;
                    }
                    globle_path_initial.points.push_back(p);
                    pub_to_control_all.publish(globle_path_initial);
                }
            }
            else if (start_pub && end_pub)
            {
                if (end_pub_finish && start_pub_finish)
                {
                    pub_flag = 0;
                    end_pub = 0;
                    end_pub_finish = 0;
                    start_pub = 0;
                    start_pub_finish = 0;
                    by_global_path_planning::Path globle_path_initial;
                    for (size_t j = 0; j < track_road.size(); j++)
                    {
                        by_global_path_planning::PathPoint p;
                        p.x = track_road[j].x;
                        p.y = track_road[j].y;
                        p.l = track_road[j].l;
                        p.r = track_road[j].r;
                        globle_path_initial.points.push_back(p);
                    }
                    by_global_path_planning::PathPoint p;
                    p.x = end_point_2.x;
                    p.y = end_point_2.y;
                    p.heading = end_point_2.theta;
                    if (!globle_path_initial.points.empty())
                    {
                        p.l = globle_path_initial.points.back().l; 
                        p.r = globle_path_initial.points.back().r; 
                    }
                    else
                    {
                        p.l = l_default_; 
                        p.r = 0.0;
                    }
                    globle_path_initial.points.push_back(p);
                    pub_to_control_all.publish(globle_path_initial);
                }
            }
        }
    }
}
void map_control::astar_start_callback(const nav_msgs::Path msg)
{
    if (dis_start)
    {
        dis_start = 0;
        track_road_1.clear();
        track_road_1_b.clear();
        track_road_1_new.clear();
        first_flag_1 = 0;
        for (int i = 1; i < (msg.poses.size() - 2); i++)
        {
            Point p;
            p.x = msg.poses[i].pose.position.x;
            p.y = msg.poses[i].pose.position.y;
            p.l = l_default_; 
            track_road_1.push_back(p);
        }
        first_flag_1 = 1;
        track_road_1_new = low_pass_filter_road_2(track_road_1, 25, 10);
        rviz_road(this->marker_pub_road_rviz_1, this->track_road_1_new, 0, 1, 0); 
    }
}
void map_control::astar_start_callback_2(const nav_msgs::Path msg)
{
    if (end_1)
    {
        end_1 = 0;
        track_road_2.clear();
        track_road_2_b.clear();
        track_road_2_new.clear();
        first_flag_2 = 0;
        for (int i = 1; i < (msg.poses.size() - 1); i++)
        {
            Point p;
            p.x = msg.poses[i].pose.position.x;
            p.y = msg.poses[i].pose.position.y;
            p.l = l_default_; 
            track_road_2.push_back(p);
        }
        first_flag_2 = 1;
        track_road_2_new = low_pass_filter_road_2(track_road_2, 25, 10);
        rviz_road(this->marker_pub_road_rviz_2, this->track_road_2_new, 0, 0, 1); 
    }
}
void map_control::end_point_callback(const geometry_msgs::PoseStamped::ConstPtr msg)
{
    Point end_point;
    end_point.x = msg->pose.position.x;
    end_point.y = msg->pose.position.y;
    double raw, pitch, theta;
    tf::Quaternion q;
    tf::quaternionMsgToTF(msg->pose.orientation, q);
    tf::Matrix3x3(q).getRPY(raw, pitch, theta);
    end_point.theta = theta;
    end_point_2 = end_point;
    int wait_count = 0;
    const int max_wait_count = 20;  
    received_nearest_goal_ = false; 
    while (!received_nearest_goal_ && wait_count < max_wait_count)
    {
        // ROS_INFO("等待nearest_goal_node_更新...（%d/%d）", wait_count + 1, max_wait_count);
        ros::Duration(0.1).sleep(); 
        ros::spinOnce(); 
        wait_count++;
    }
    if (!received_nearest_goal_ || std::isnan(nearest_goal_node_.x) || std::isnan(nearest_goal_node_.y))
    {
        ROS_WARN("未收到有效的nearest_goal_node_，跳过终点A*规划");
        return;
    }
    double dist_to_node = sqrt(pow(end_point.x - nearest_goal_node_.x, 2) + pow(end_point.y - nearest_goal_node_.y, 2));
    double node_dist = sqrt(pow(nearest_start_node_.x - nearest_goal_node_.x, 2) + pow(nearest_start_node_.y - nearest_goal_node_.y, 2));
    if (node_dist < 0.1)
    {
        return; 
    }
    if (dist_to_node > 4.0 && start_only_a_single_astar_path == false) 
    {
        dis_end = 1;
        end_1 = 1;
        end_pub = 1;
        geometry_msgs::PoseWithCovarianceStamped start_2;
        start_2.header.stamp = ros::Time::now();
        start_2.header.frame_id = "odom";
        start_2.pose.pose.position.x = nearest_goal_node_.x;
        start_2.pose.pose.position.y = nearest_goal_node_.y;
        start_2.pose.pose.orientation = tf::createQuaternionMsgFromYaw(0);
        pub_astar_start_2.publish(start_2);
        geometry_msgs::PoseStamped target_2;
        target_2.header.stamp = ros::Time::now();
        target_2.header.frame_id = "world";
        target_2.pose.position.x = end_point.x;
        target_2.pose.position.y = end_point.y;
        target_2.pose.orientation = msg->pose.orientation;
        pub_astar_end_2.publish(target_2);
    }
    first == false; 
}

std::vector<Point> map_control::low_pass_filter_road_2(vector<Point> &roads, int num, float max_dis)
{
    vector<Point> road_points_new; 
    if (roads.empty()) 
    {
        ROS_WARN("low_pass_filter_road_2: 输入路径为空，直接返回");
        return road_points_new;
    }
    if (roads.size() == 1)
    {
        if (std::isnan(roads[0].x) || std::isnan(roads[0].y))
        {
            ROS_WARN("low_pass_filter_road_2: 单点坐标无效");
            return road_points_new;
        }
        road_points_new.push_back(roads[0]);
        return road_points_new;
    }
    Point p_last;
    p_last.x = roads[0].x; 
    p_last.y = roads[0].y;
    p_last.l = roads[0].l;
    p_last.r = roads[0].r;
    int count_dis = 0;
    for (int i = 0; i < roads.size(); i++)
    {
        Point p;
        p.x = roads[i].x;
        p.y = roads[i].y;
        p.l = roads[i].l;
        p.r = roads[i].r;
        float dis = sqrt(pow(p.x - p_last.x, 2) + pow(p.y - p_last.y, 2));
        if (dis < max_dis)
        {
            road_points_new.push_back(p);
            p_last.x = p.x;
            p_last.y = p.y;
            p_last.l = p.l;
            p_last.r = p.r;
            count_dis = 0;
        }
        else
        {
            if (count_dis < 5)
            {
                count_dis++;
            }
            else
            {
                p_last.x = p.x;
                p_last.y = p.y;
                p_last.l = p.l;
                p_last.r = p.r;
            }
        }
    }
    int count = 0;
    float x_sum = 0, y_sum = 0;
    vector<Point> road_points_new_new;
    Point p_start;
    p_start.x = road_points_new[0].x;
    p_start.y = road_points_new[0].y;
    p_start.l = road_points_new[0].l;
    p_start.r = road_points_new[0].r;
    road_points_new_new.push_back(p_start);
    for (int i = 1; i < road_points_new.size() - 1; i++)
    {
        if (count < num)
        {
            x_sum += road_points_new[i].x;
            y_sum += road_points_new[i].y;
            count++;
        }
        else
        {
            Point p;
            p.x = x_sum / (float)count;
            p.y = y_sum / (float)count;
            p.l = road_points_new[i].l;
            p.r = road_points_new[i].r;
            y_sum = 0;
            x_sum = 0;
            road_points_new_new.push_back(p);
            count = 0;
        }
    }
    Point p_end;
    p_end.x = road_points_new[road_points_new.size() - 1].x;
    p_end.y = road_points_new[road_points_new.size() - 1].y;
    p_end.l = road_points_new[road_points_new.size() - 1].l;
    p_end.r = road_points_new[road_points_new.size() - 1].r;
    road_points_new_new.push_back(p_end);
    vector<Point> road_store;
    for (int i = 0; i < road_points_new_new.size(); i++)
    {
        Point p;
        p.x = road_points_new_new[i].x;
        p.y = road_points_new_new[i].y;
        p.l = road_points_new_new[i].l;
        p.r = road_points_new_new[i].r;
        road_store.push_back(p);
    }
    return road_store;
}
void map_control::node_start(int argc, char *argv[])
{
    ros::init(argc, argv, "map_control");
    ros::NodeHandle n;
    int odom_ = -1; 
    int slam_ = -1;
    int gps_ = -1;
    n.param<int>("ODOM_", odom_, -1);
    n.param<int>("SLAM_", slam_, -1);
    n.param<int>("GPS_", gps_, -1);
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

    // 发布方实现
    // 1.给pure_pursuit提供实际追踪的路径
    pub_to_control_all = n.advertise<by_global_path_planning::Path>("to_control_all", 10);
    // 2.确定出来一个函数，即如果判断dis_start的标志位为1，即发布两个位置，一个是车的位置为起点，一个是离车最近的线上的点为终点，将两个点发布出去
    pub_astar_start_1 = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose_1", 10);
    // pub_astar_end_1 = n.advertise<geometry_msgs::PoseStamped>("move_base_simple/goal_1", 1);
    pub_astar_end_1 = n.advertise<geometry_msgs::PoseStamped>("move_base_simple/goal_1", 10);
    // 3.确定出来另一个函数，即如果判断dis_end的标志位为1，即发布两个位置，一个是离车最近的线上的点为起点，一个是车的位置为终点，将两个点换另一个话题发布出去
    pub_astar_start_2 = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose_2", 10);
    pub_astar_end_2 = n.advertise<geometry_msgs::PoseStamped>("move_base_simple/goal_2", 10);
    // 4.单a星
    pub_astar_start_3 = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose_3", 1); 
    pub_astar_end_3 = n.advertise<geometry_msgs::PoseStamped>("move_base_simple/goal_3", 1);    
    // 5.根据规划出来的道路，在rviz中进行展示
    marker_pub_road_rviz_1 = n.advertise<visualization_msgs::Marker>("road_rviz_1", 1);
    marker_pub_road_rviz_2 = n.advertise<visualization_msgs::Marker>("road_rviz_2", 1);
    marker_pub_road_rviz_3 = n.advertise<visualization_msgs::Marker>("road_rviz_3", 1); 
    
    // 订阅方实现
    // 4.接受A星发布过来的两条路径，根据话题名不同，都要首先进行滤波，然后放进对应的路径中，有可能是开头，有可能是结尾
    // 1.接受djstl发过来的全局路径,全局路径中如果终点在线外，最后一个点是终点信息，如果起点在线外，第一个点是最近点的信息。将djstl规划出来的路径放到track_road中
    ros::Subscriber get_control = n.subscribe("to_control", 1, &map_control::get_control_callback, this);
    // 2.接受odom或者slam发过来的定位信息,赋值给车所处的实时位置，判断此时车的位置离djstl发过来的轨迹最近的点是否>3米，如果大于，dis_start为1，直接读存储的路线即可
    ros::Subscriber sub_slam = n.subscribe("/localization", 1, &map_control::odometryGetCallBack_slam, this);
    ros::Subscriber sub_odom = n.subscribe("/odom", 1, &map_control::odometryGetCallBack, this);
    // 3.计时器中断，每一秒钟进入一次，实时读取参数服务器中floor的数值，并从txt文件中读取到整体地图数据并存在roads中
    // ros::Timer timer1 = n.createTimer(ros::Duration(1), &map_control::timer_cb1, this); // 现在不适用定时器了，因此需注释掉，不然编译失败
    // 4.接收到终点的信息后,进行判断终点与道路上离终点最近的点的信息，如果大于五米，则进行A*处理
    ros::Subscriber sub_end_point = n.subscribe("/move_base_simple/goal", 10, &map_control::end_point_callback, this);
    // 5.订阅A星发过来的两条路径，根据话题名不同，都要首先进行滤波，然后放进对应的路径中，有可能是开头，有可能是结尾
    ros::Subscriber sub_astar_start = n.subscribe("/nav_path", 10, &map_control::astar_start_callback, this);
    ros::Subscriber sub_astar_start_2 = n.subscribe("/nav_path_2", 10, &map_control::astar_start_callback_2, this);
    sub_astar_path_3 = n.subscribe("/nav_path_3", 10, &map_control::astarPath3Callback, this);
    // 6.最/次近节点
    sub_nearest_start_node_ = n.subscribe("/nearest_start_node", 1, &map_control::nearestStartNodeCallback, this);
    sub_nearest_goal_node_ = n.subscribe("/nearest_goal_node", 1, &map_control::nearestGoalNodeCallback, this);

    while (ros::ok)
    {
        ros::spinOnce();
    }
}
int main(int argc, char *argv[])
{
    setlocale(LC_CTYPE, "zh_CN.utf8");
    map_control node;
    node.node_start(argc, argv);
    return 0;
}