#include <vector>
#include <iostream>
#include <cmath>          // 包含sin和cos函数
#include <Eigen/Geometry> // 包含四元数和变换的头文件
#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <geometry_msgs/Twist.h>
#include <jsk_recognition_msgs/BoundingBox.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Bool.h>
#include <tf/tf.h>

#include "lidar_ros/Box2d.h"
// 下面两个头文件时根据 自定义msg 消息自动生成的
#include "lidar_ros/Path.h"
#include "lidar_ros/PathPoint.h"
#include "lidar_ros/road_detection.h"
#include "by_global_path_planning/Path.h"
#include "by_global_path_planning/PathPoint.h"

struct State
{
    double x = 0;     // m
    double y = 0;     // m
    double yaw = 0;   // 弧度
    double speed = 0; // m/s
    double yawrate = 0;
};

double calcDistance(const Point &start, const State &end)
{
    double x = end.x - start.x;
    double y = end.y - start.y;
    return sqrt(x * x + y * y);
};

int FindNearestPoint(std::vector<Point> path, State veh)
{
    double distance;
    Point first_point;
    first_point.x = path[0].x;
    first_point.y = path[0].y;
    double min_distance = calcDistance(first_point, veh);
    int result = 0;
    for (int i = 0; i < path.size(); i++)
    {
        Point point_;
        point_.x = path[i].x;
        point_.y = path[i].y;
        distance = calcDistance(point_, veh);
        if (distance < min_distance)
        {
            result = i;
            min_distance = distance;
        }
    }
    return result;
}
State vehicleState;
std::vector<Point> track_road;
std::vector<Point> track_road_3;
int floor_de = 0;
int floor_last_de = 0;
// vector<Road> roads_third; 
/*----------------------------------------------------------------------------- */

const double ago_distance_th = 0.8;   // 前方距离阈值
const double after_distance_th = 0.2; // 后方距离阈值
const double left_distance_th = 0.1;  // 左侧距离阈值
const double right_distance_th = 0.1; // 右侧距离阈值, 左侧和右侧最好相同
const double car_length = 1.8;       // 车辆长度
const double car_width = 0.9;         // 车辆宽度
// 包含安全距离的 car_box
const double car_length_update = car_length + ago_distance_th + after_distance_th;              // 车辆安全范围长度
const double car_length_update_floor8 = car_length + ago_distance_th + after_distance_th - 0.5; // 车辆安全范围长度，前后均为0.5m
const double car_width_update = car_width + left_distance_th + right_distance_th;               // 车辆安全范围宽度

const double car_center_x_00 = (ago_distance_th - after_distance_th) / 2 - car_length / 2;
const double car_center_y_00 = (left_distance_th - right_distance_th) / 2;

jsk_recognition_msgs::BoundingBox return_2Dbox_msgs(std::string frame_id, double x, double y, double radian, double length, double width)
{

    // 使用Eigen::AngleAxisd创建旋转对象
    Eigen::AngleAxisd rotation_vector(radian, Eigen::Vector3d::UnitZ());
    // 将旋转对象转换为四元数
    Eigen::Quaterniond quaternion(rotation_vector);

    jsk_recognition_msgs::BoundingBox box;
    box.header.frame_id = frame_id;
    box.pose.position.x = x;
    box.pose.position.y = y;
    box.pose.position.z = 0;

    box.pose.orientation.x = quaternion.x();
    box.pose.orientation.y = quaternion.y();
    box.pose.orientation.z = quaternion.z();
    box.pose.orientation.w = quaternion.w();

    box.dimensions.x = length;
    box.dimensions.y = width;
    box.dimensions.z = 0.1;

    return box;
}

void obstacle_Callback(const std_msgs::Float64MultiArray::ConstPtr &msg, ros::Publisher pub_stop_lidar, ros::Publisher pub_box, ros::Publisher pub_box_array, ros::Publisher pub_is_avoidance)
{
    ROS_INFO("--------------------------------------------");

    std::vector<double> obstacles = msg->data;
    // 检查是否至少有一个完整的障碍物信息（10个数据）
    if (obstacles.size() % 10 != 0)
    {
        ROS_ERROR("Invalid obstacle data format!");
        return;
    }

    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = 0;
    cmd_vel.linear.y = 0;
    cmd_vel.angular.z = 0;

    ros::Rate rate(10);

    jsk_recognition_msgs::BoundingBoxArray box_array;
    box_array.header.frame_id = "map"; // map

    std_msgs::Bool is_avoidance_msg;
    is_avoidance_msg.data = 0;

    ROS_WARN("----------START!----------");
    // 判断当前车辆的道路宽度，只有当前道路宽度等于l
    // from li 一楼道路①l=r=3 道路②l=r=4 道路③l=r=4
    // from li 三楼道路①l=r=0 道路②l=r=4 道路③l=r=4
    if (track_road.size() > 0)
    {

        int road_idx = FindNearestPoint(track_road, vehicleState);
        // int road_idx_3 = FindNearestPoint(track_road_3, vehicleState);
        // cout << "-------------road--------------" << road_idx_3 << endl;
        // 车辆位于道路2、3/对应l为4，全程开启避障
        // 0为既不开停障、也不开避障：
        if (track_road[road_idx].l == 0) // 车辆位于道路2 // 此时应用在25年6月5日汉韵公馆第二条路线上，也就是进入电梯井内部道路
        {
            // ROS_WARN("道路ID: 白云机场出电梯3楼1路");
            ROS_WARN("l 0 ——> 既不开停障、也不开避障");
            ros::param::set("stop_lidar", 0);
            ros::param::set("shifoubizhang", false);
        }

        // 1为只开停障、不开避障： 
        if (track_road[road_idx].l == 1) // 车辆位于白云机场三楼的第二第三条路上，此时只会停障，不会避障 
        { 
            ROS_WARN("l 1 ——> 只停障");
            // ROS_WARN("道路ID: 2/3");
            // 此时应用在25年6月5日汉韵公馆第三条路线上
            ros::param::set("shifoubizhang", false); 

            //  提取障碍物信息
            for (size_t i = 0; i < obstacles.size(); i += 10)
            {
                // 提取中心坐标
                double obstacle_center_x = obstacles[i];
                double obstacle_center_y = obstacles[i + 1];
                double obstacle_center_z = obstacles[i + 2];

                // 提取长宽高坐标
                double obstacle_length = obstacles[i + 6];
                double obstacle_width = obstacles[i + 7];
                double obstacle_height = obstacles[i + 8];

                // 提取角度,!!!!要弧度
                double obstacle_heading = obstacles[i + 9]; // 弧度制

                // 打印提取到的障碍物信息
                ROS_INFO("障碍物中心坐标：(%.2f, %.2f, %.2f), 长宽高：(%.2f, %.2f, %.2f), 朝向角(弧度): %.2f",
                         obstacle_center_x, obstacle_center_y, obstacle_center_z,
                         obstacle_length, obstacle_width, obstacle_height, obstacle_heading);

                // 单个障碍物和车辆的碰撞检测
                // 车辆box
                double car_center_update_x = vehicleState.x;
                double car_center_update_y = vehicleState.y;
                double car_heading = vehicleState.yaw;
                Vec2d car_center(car_center_update_x, car_center_update_y); // 车辆中心
                Box_2d car_box(car_center, car_heading, car_length_update, car_width_update, 0);

                // 障碍物box
                Vec2d obstacle_center(obstacle_center_x, obstacle_center_y);
                Box_2d obstacle_box(obstacle_center, obstacle_heading, obstacle_length, obstacle_width, 0);

                // rviz 可视化
                // 障碍物
                jsk_recognition_msgs::BoundingBox obstacle_box_msgs = return_2Dbox_msgs("map", obstacle_center_x, obstacle_center_y, obstacle_heading, obstacle_length, obstacle_width);
                box_array.boxes.push_back(obstacle_box_msgs);
                pub_box_array.publish(box_array);
                // car
                jsk_recognition_msgs::BoundingBox car_box_msgs = return_2Dbox_msgs("map", car_center_update_x, car_center_update_y, car_heading, car_length_update, car_width_update);
                pub_box.publish(car_box_msgs);

                bool result = car_box.HasOverlap(obstacle_box); // HasOverlap 函数返回false代表无碰撞
                std::cout << "result:" << result << std::endl;

                int right_rader_stop = 0;
                int left_rader_stop = 0;
                ros::param::get("rader_right_stop", right_rader_stop);
                ros::param::get("rader_left_stop", left_rader_stop);
                if (result == true || right_rader_stop == 1 || left_rader_stop == 1)
                {
                    ROS_WARN("停车! ");

                    ros::param::set("stop_lidar", 1);

                    // 向 cmd_vel 发送速度控制
                    pub_stop_lidar.publish(cmd_vel);

                    break;
                }

                // ros::param::set("stop_lidar", 0); // 发送过于随意
                int stop_number = 0;
                ros::param::get("stop_lidar_number", stop_number);
                stop_number = stop_number + 1;
                ros::param::set("stop_lidar_number", stop_number);
                if (stop_number >= 7)
                {
                    ros::param::set("stop_lidar", 0); // 发送过于随意
                    ros::param::set("stop_lidar_number", 0);
                }
            }
        }

        // 2为只开避障、不开停障：
        if (track_road[road_idx].l == 2)
        {
            ROS_WARN("l 2 ——> 只避障");
            ros::param::set("stop_lidar", 0);
            ros::param::set("shifoubizhang", true);
        }
    }

    rate.sleep();
    static int stop = 0;
    ros::param::get("stop_lidar", stop);
    ROS_WARN("stop: %d", stop);
}

void vehicle_Callback(const nav_msgs::Odometry::ConstPtr odometry_msg)
{

    double roll, pitch, yaw;
    tf::Quaternion q;
    // 将四元数从ROS消息格式 转换为 TF四元数格式
    tf::quaternionMsgToTF(odometry_msg->pose.pose.orientation, q);
    // 四元数 -》 旋转矩阵 -》 欧拉角
    tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
    vehicleState.yaw = yaw;
    vehicleState.x = odometry_msg->pose.pose.position.x - cos(yaw) * abs(car_center_x_00);
    vehicleState.y = odometry_msg->pose.pose.position.y - sin(yaw) * abs(car_center_x_00);
}

void get_control_callback(const by_global_path_planning::Path msg)
{
    ROS_INFO("in get_control_callback");
    ROS_WARN("in get_control_callback");
    track_road.clear();
    track_road_3.clear();
    for (int i = 0; i < msg.points.size(); i++)
    {
        Point p;
        p.x = msg.points[i].x;
        p.y = msg.points[i].y;
        p.theta = msg.points[i].heading;
        p.l = msg.points[i].l;
        p.r = msg.points[i].r;
        track_road.push_back(p);
        ros::param::set("pppp", p.x);
    }

    for (int i = 0; i < track_road.size(); ++i)
    {
        ros::param::set("iiii", i);
        std::cout << "00000000000000000000: " << track_road[i].x << std::endl;
    }
    int my_size = track_road.size();
    ros::param::set("size", my_size);

    int sanlou_2 = 1;
    ros::param::set("sanlou_2", sanlou_2);
    // ros::param::get("floor", floor_de);
    // if (floor_de == 1 && floor_de != floor_last_de)
    // {
    //     string roadMap_path = "/home/yt/agv/data/hygg_floor_1.txt"; // 地图录制位置straight  slam_road123
    //     roads_third = load_direction_Road_3(roadMap_path);
    //     floor_last_de = floor_de;
    // }
    // else if (floor_de == 2 && floor_de != floor_last_de)
    // {
    //     string roadMap_path = "/home/yt/agv/data/hygg_floor_2.txt"; // 地图录制位置straight  slam_road123
    //     roads_third = load_direction_Road_3(roadMap_path);
    //     floor_last_de = floor_de;
    // }
    // else if (floor_de == 4 && floor_de != floor_last_de)
    // {
    //     string roadMap_path = "/home/yt/agv/data/hygg_floor_4.txt"; // 地图录制位置straight  slam_road123
    //     roads_third = load_direction_Road_3(roadMap_path);
    //     floor_last_de = floor_de;
    // }
    // else if (floor_de == 6 && floor_de != floor_last_de)
    // {
    //     string roadMap_path = "/home/yt/agv/data/hygg_floor_6.txt"; // 地图录制位置straight  slam_road123
    //     roads_third = load_direction_Road_3(roadMap_path);
    //     floor_last_de = floor_de;
    // }
    // else if (floor_de == 8 && floor_de != floor_last_de)
    // {
    //     string roadMap_path = "/home/yt/agv/data/hygg_floor_8.txt";
    //     roads_third = load_direction_Road_3(roadMap_path);
    //     floor_last_de = floor_de;
    // }
    // else if (floor_de == 10 && floor_de != floor_last_de)
    // {
    //     string roadMap_path = "/home/yt/agv/data/hygg_floor_10.txt";
    //     roads_third = load_direction_Road_3(roadMap_path);
    //     floor_last_de = floor_de;
    // }
    // else if (floor_de == 12 && floor_de != floor_last_de)
    // {
    //     string roadMap_path = "/home/yt/agv/data/hygg_floor_12.txt";
    //     roads_third = load_direction_Road_3(roadMap_path);
    //     floor_last_de = floor_de;
    // }
    // else if (floor_de == 14 && floor_de != floor_last_de)
    // {
    //     string roadMap_path = "/home/yt/agv/data/hygg_floor_14.txt";
    //     roads_third = load_direction_Road_3(roadMap_path);
    //     floor_last_de = floor_de;
    // }
    // else if (floor_de == 16 && floor_de != floor_last_de)
    // {
    //     string roadMap_path = "/home/yt/agv/data/hygg_floor_16.txt";
    //     roads_third = load_direction_Road_3(roadMap_path);
    //     floor_last_de = floor_de;
    // }
    // else if (floor_de == 18 && floor_de != floor_last_de)
    // {
    //     string roadMap_path = "/home/yt/agv/data/hygg_floor_18.txt";
    //     roads_third = load_direction_Road_3(roadMap_path);
    //     floor_last_de = floor_de;
    // }
    // else if (floor_de == 20 && floor_de != floor_last_de)
    // {
    //     string roadMap_path = "/home/yt/agv/data/hygg_floor_20.txt";
    //     roads_third = load_direction_Road_3(roadMap_path);
    //     floor_last_de = floor_de;
    // }
    // else if (floor_de == 22 && floor_de != floor_last_de)
    // {
    //     string roadMap_path = "/home/yt/agv/data/hygg_floor_22.txt";
    //     roads_third = load_direction_Road_3(roadMap_path);
    //     floor_last_de = floor_de;
    // }
    // else if (floor_de == 24 && floor_de != floor_last_de)
    // {
    //     string roadMap_path = "/home/yt/agv/data/hygg_floor_24.txt";
    //     roads_third = load_direction_Road_3(roadMap_path);
    //     floor_last_de = floor_de;
    // }
    // else if (floor_de == 26 && floor_de != floor_last_de)
    // {
    //     string roadMap_path = "/home/yt/agv/data/hygg_floor_26.txt";
    //     roads_third = load_direction_Road_3(roadMap_path);
    //     floor_last_de = floor_de;
    // }
    // else if (floor_de == 28 && floor_de != floor_last_de)
    // {
    //     string roadMap_path = "/home/yt/agv/data/hygg_floor_28.txt";
    //     roads_third = load_direction_Road_3(roadMap_path);
    //     floor_last_de = floor_de;
    // }
    // else if (floor_de == 30 && floor_de != floor_last_de)
    // {
    //     string roadMap_path = "/home/yt/agv/data/hygg_floor_30.txt";
    //     roads_third = load_direction_Road_3(roadMap_path);
    //     floor_last_de = floor_de;
    // }
    // else if (floor_de == 99 && floor_de != floor_last_de)
    // {
    //     string roadMap_path = "/home/yt/agv/data/hygg_floor_1.txt";
    //     roads_third = load_direction_Road_3(roadMap_path);
    //     floor_last_de = floor_de;
    // }

    // for (int i = 0; i < roads_third[0].road_points.size(); i++)
    // {
    //     Point p_3;
    //     p_3.x = roads_third[0].road_points[i].x;
    //     p_3.y = roads_third[0].road_points[i].y;
    //     p_3.theta = roads_third[0].road_points[i].theta;
    //     p_3.l = roads_third[0].road_points[i].l;
    //     p_3.r = roads_third[0].road_points[i].r;
    //     track_road_3.push_back(p_3);
    // }
    std::cout << "1010000302324235426436" << std::endl;
}

int main(int argc, char **argv)
{

    setlocale(LC_ALL, "");
    ros::init(argc, argv, "collision_detection_node");

    ros::param::set("stop_lidar", 0);
    ros::param::set("bizhang_stop", 0);
    ros::param::set("stop_lidar_number", 0);

    ros::NodeHandle nh;
    ros::Publisher pub_stop_lidar = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10); // 发布cmd_vel
    ros::Publisher pub_box = nh.advertise<jsk_recognition_msgs::BoundingBox>("/lidar/2D_box", 5);
    ros::Publisher pub_box_array = nh.advertise<jsk_recognition_msgs::BoundingBoxArray>("/lidar/2D_boxs_array", 10);
    ros::Publisher pub_is_avoidance = nh.advertise<std_msgs::Bool>("/is_avoidance", 10);

    // ros::Subscriber sub_get_control = nh.subscribe<by_global_path_planning::Path>("/to_control", 1, get_control_callback); // 接受local规划路径
    ros::Subscriber sub_get_control = nh.subscribe<by_global_path_planning::Path>("/to_control_all", 1, get_control_callback);
    
    ros::Subscriber sub_slam = nh.subscribe<nav_msgs::Odometry>("/localization", 1, vehicle_Callback);
    ros::Subscriber sub_obstacle = nh.subscribe<std_msgs::Float64MultiArray>("/lidar/map_detect_result", 1,
                                                                             boost::bind(obstacle_Callback, _1, pub_stop_lidar, pub_box, pub_box_array, pub_is_avoidance));

    while (ros::ok)
    {
        ros::Time time_now = ros::Time::now();
        ros::spinOnce();
        ros::Time time_now_2 = ros::Time::now();
        std::cout << "-----------collision spin cost:  " << time_now_2 - time_now << std::endl;
    }

    return 0;
}