#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/GetMap.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/Float64MultiArray.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <path_planning_msgs/PathPoint.h>
#include <path_planning_msgs/Path.h>
#include <by_global_path_planning/PathPoint.h>
#include <by_global_path_planning/Path.h>
#include <vector>
#include <cmath>
#include <algorithm>
#include <limits>
#include <queue>
#include <string>
#include <std_msgs/String.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Bool.h> // 添加此头文件
///////////////////////////////////////////////////
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

struct CustomObstacle
{
    double x, y; // 中心点
    double length, width;
    double theta; // 朝向（弧度）

    CustomObstacle(double x_, double y_, double l_, double w_, double t_ = 0)
        : x(x_), y(y_), length(l_), width(w_), theta(t_) {}
};

struct Pose
{
    double x, y, theta;

    Pose() : x(0), y(0), theta(0) {}
    Pose(double x_, double y_, double theta_) : x(x_), y(y_), theta(theta_) {}
};

struct Velocity
{
    double v, w; // 线速度和角速度

    Velocity() : v(0), w(0) {}
    Velocity(double v_, double w_) : v(v_), w(w_) {}
};

struct DWAConfig
{
    // 速度限制
    double max_v = 1.0;
    double min_v = 0.0;
    double max_w = 15.0;  // 原5.0，0924改为15.0
    double min_w = -15.0; // 原-5.0，0924改为-15.0

    // 加速度限制
    double max_accel_v = 0.6;
    double max_accel_w = 5.0; // 原3，0924改为5.0

    // 分辨率
    double v_resolution = 0.1;
    double w_resolution = 0.025;

    // 仿真参数
    double dt = 0.1;
    double predict_time = 9.0;

    // 代价函数权重
    double to_goal_cost_gain = 0.7; // 原0.5，0924改为0.7
    double speed_cost_gain = 0.1;
    double obstacle_cost_gain = 3.0;
    double path_cost_gain = 0.2; // 原0.5，0924改为0.2

    // 机器人参数
    double robot_length = 1.8;
    double robot_width = 0.9;

    // 安全距离
    double safe_dist = 0.1;      // 与障碍物的安全距离
    double goal_tolerance = 0.5; // 目标容忍度
};

struct TrajectoryScore
{
    double to_goal_cost;
    double speed_cost;
    double obstacle_cost;
    double path_cost;
    double total_cost;

    TrajectoryScore() : to_goal_cost(0), speed_cost(0), obstacle_cost(0), path_cost(0), total_cost(0) {}
};

class DWALocalPlanner
{
private:
    ros::NodeHandle nh_;
    ros::Publisher cmd_vel_pub_;
    ros::Publisher trajectory_pub_;
    ros::Publisher obstacles_pub_;
    ros::Publisher local_map_pub_;
    ros::Publisher car_pub_;
    ros::Publisher local_path_pub_;

    ros::Subscriber sub_slam;
    ros::Subscriber sub_obs;
    ros::Subscriber sub_ready_to_avoidance;
    ros::Subscriber odom_sub_;
    ros::Subscriber path_sub_;
    ros::Subscriber custom_obstacles_sub_;
    ros::Subscriber sub_globle_path;
    ros::Subscriber get_avoidance_control;
    ros::ServiceClient map_client_;
    ros::Timer map_request_timer_;
    ros::Timer control_timer_;
    ros::Subscriber pointcloud_sub_;

    nav_msgs::OccupancyGrid global_map_;
    nav_msgs::OccupancyGrid local_map_;
    nav_msgs::Path global_path_;
    Pose current_pose_;
    Velocity current_vel_;
    std::vector<CustomObstacle> custom_obstacles_;
    double lookahead_dist_ = 5.0; // 前视距离，可通过参数加载

    DWAConfig config_;
    bool map_received_;
    bool path_received_;
    bool odom_received_;
    bool ls_avoidance_ = false;
    bool ls_ready_to_avoidance_ = false;

    // 新增参数
    double obstacle_map_range_ = 5.0; // 只考虑5米内障碍 // 原6.0，0924改为5.0
    double roi_radius_ = 6.0;         // ROI半径

    // DWA相关变量
    std::vector<std::vector<Pose>> all_trajectories_;
    std::vector<TrajectoryScore> trajectory_scores_;

public:
    DWALocalPlanner() : nh_("~"), map_received_(false), path_received_(false), odom_received_(false)
    {
        // 初始化发布者
        cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
        trajectory_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/dwa_trajectories", 1);
        car_pub_ = nh_.advertise<visualization_msgs::Marker>("/car", 1);
        obstacles_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/custom_obstacles2", 1);
        local_map_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("/local_map", 1);
        local_path_pub_ = nh_.advertise<nav_msgs::Path>("/local_path", 1);

        // 初始化订阅者
        // sub_obs = nh_.subscribe("/lidar/map_detect_result", 10, &DWALocalPlanner::obstacle_GetCallBack, this);//接收检测到的障碍物
        sub_ready_to_avoidance = nh_.subscribe("/ready_to_obstacle", 1, &DWALocalPlanner::ready_to_avoidance_callback, this); // 接受避障
        // odom_sub_ = nh_.subscribe("/odom", 1, &DWALocalPlanner::odomCallback, this);
        path_sub_ = nh_.subscribe("/global_path", 1, &DWALocalPlanner::pathCallback, this);
        custom_obstacles_sub_ = nh_.subscribe("/custom_obstacles", 10, &DWALocalPlanner::customObstaclesCallback, this);
        sub_globle_path = nh_.subscribe("/to_control_all", 1, &DWALocalPlanner::globlepathGetCallBack, this);  // 接受全局规划路径
        get_avoidance_control = nh_.subscribe("/is_avoidance", 1, &DWALocalPlanner::avoidance_callback, this); // 接受避障
        sub_slam = nh_.subscribe("/localization", 1, &DWALocalPlanner::odometryGetCallBack_slam, this);        // 接收的是slam的信息
        pointcloud_sub_ = nh_.subscribe("/lidar/filter_pointCloud_map", 2, &DWALocalPlanner::pointCloudCallback, this);

        nh_.param("roi_radius", roi_radius_, 6.0);
        nh_.param("obstacle_map_range", obstacle_map_range_, 6.0);
        // 地图服务
        std::string map_service_name;
        nh_.param("map_service", map_service_name, std::string("/static_map"));
        map_client_ = nh_.serviceClient<nav_msgs::GetMap>(map_service_name);

        // // 加载参数
        // loadParameters();

        // 初始化自定义障碍物（示例）
        // initCustomObstacles();

        // 延迟请求地图
        map_request_timer_ = nh_.createTimer(ros::Duration(2.0),
                                             &DWALocalPlanner::mapRequestTimerCallback,
                                             this, true);

        // 控制循环定时器
        control_timer_ = nh_.createTimer(ros::Duration(0.1),
                                         &DWALocalPlanner::controlLoop, this);

        ROS_INFO("DWA Local Planner initialized");
    }

    ////////////////////////////////////////////////////////////////////////////////////////////
    void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr &cloud_msg)
    {
        // 转PCL格式
        pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*cloud_msg, *pcl_cloud);

        // 取6m范围内点
        std::vector<std::pair<int, int>> grid_indices; // 地图上的点
        double robot_x = current_pose_.x;
        double robot_y = current_pose_.y;
        double roi_radius = 6.0; // 只处理6m内

        // 先清空局部地图，copy全局地图或者直接初始化free
        local_map_ = global_map_;
        std::fill(local_map_.data.begin(), local_map_.data.end(), 0); // 0 free

        // 地图参数
        double res = local_map_.info.resolution;
        int map_w = local_map_.info.width;
        int map_h = local_map_.info.height;
        double ox = local_map_.info.origin.position.x;
        double oy = local_map_.info.origin.position.y;

        // 处理点云
        for (const auto &pt : pcl_cloud->points)
        {
            double dx = pt.x - robot_x;
            double dy = pt.y - robot_y;
            if (dx * dx + dy * dy > roi_radius * roi_radius)
                continue; // 超出ROI

            int gx = (int)((pt.x - ox) / res);
            int gy = (int)((pt.y - oy) / res);
            if (gx >= 0 && gx < map_w && gy >= 0 && gy < map_h)
            {
                int idx = gy * map_w + gx;
                local_map_.data[idx] = 100; // 障碍
                grid_indices.push_back({gx, gy});
            }
        }

        // 膨胀
        int inflate = (int)(config_.safe_dist / res);       // 以安全距离为半径膨胀
        std::vector<int8_t> inflated_map = local_map_.data; // copy
        for (const auto &idx_pair : grid_indices)
        {
            int gx = idx_pair.first, gy = idx_pair.second;
            for (int dx = -inflate; dx <= inflate; ++dx)
            {
                for (int dy = -inflate; dy <= inflate; ++dy)
                {
                    int nx = gx + dx, ny = gy + dy;
                    if (nx < 0 || nx >= map_w || ny < 0 || ny >= map_h)
                        continue;
                    double dist = sqrt(dx * dx + dy * dy) * res;
                    if (dist > config_.safe_dist)
                        continue;
                    int nidx = ny * map_w + nx;
                    inflated_map[nidx] = 100;
                }
            }
        }
        local_map_.data = inflated_map;

        // 裁剪6m ROI圆
        cropLocalMapToCircularROI(robot_x, robot_y, roi_radius);

        // 发布局部代价地图（可视化）
        local_map_.header.stamp = ros::Time::now();
        local_map_.header.frame_id = "map";
        local_map_pub_.publish(local_map_);
    }

    void obstacle_GetCallBack(std_msgs::Float64MultiArray msg)
    {
        custom_obstacles_.clear();
        std::vector<double> obstacles = msg.data;
        // 提取障碍物信息
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

            // 打印提取到的障碍物信息
            ROS_INFO("障碍物中心坐标：(%.2f, %.2f, %.2f), 长宽高：(%.2f, %.2f, %.2f)",
                     obstacle_center_x, obstacle_center_y, obstacle_center_z,
                     obstacle_length, obstacle_width, obstacle_height);
            double l = obstacle_length / 2;
            double w = obstacle_width / 2;
            double r = sqrt(l * l + w * w);
            custom_obstacles_.emplace_back(obstacle_center_x, obstacle_center_y, obstacle_length, obstacle_width, r);
            updateLocalMapWithObstacles();
            publishCustomObstacles();
        }
    }

    void ready_to_avoidance_callback(const std_msgs::Bool msg)
    {
        // std::cout << "///////a/aaaa//////" << std::endl; // lzw_1114注释
        ls_ready_to_avoidance_ = msg.data;
    }

    void avoidance_callback(const std_msgs::Bool msg)
    {
        ls_avoidance_ = msg.data;
    }

    void globlepathGetCallBack(const by_global_path_planning::Path &inputPath)
    {
        // 创建 nav_msgs::Path 实例
        ROS_INFO("globlepathGetCallBack");
        nav_msgs::Path convertedPath;
        for (const auto &point : inputPath.points)
        { // 遍历所有 PathPoint
            geometry_msgs::PoseStamped pose;
            // 设置位姿的位置
            pose.pose.position.x = point.x;
            pose.pose.position.y = point.y;
            pose.pose.position.z = 0.0; // 默认 z 坐标为 0
            convertedPath.poses.push_back(pose);
        }
        // 更新全局路径并设置标志
        global_path_ = convertedPath;
        path_received_ = true;
        ROS_INFO("Converted global path with %zu waypoints", global_path_.poses.size());
    }

    geometry_msgs::Point computeLookaheadPoint()
    {
        // 1) 找 nearest index
        double min_d = 1e9;
        size_t idx = 0;
        for (size_t i = 0; i < global_path_.poses.size(); ++i)
        {
            double dx = current_pose_.x - global_path_.poses[i].pose.position.x;
            double dy = current_pose_.y - global_path_.poses[i].pose.position.y;
            double d2 = dx * dx + dy * dy;
            if (d2 < min_d)
            {
                min_d = d2;
                idx = i;
            }
        }
        // 2) 向前累计 arc-length
        double acc = 0;
        for (size_t j = idx; j + 1 < global_path_.poses.size(); ++j)
        {
            auto &p1 = global_path_.poses[j].pose.position;
            auto &p2 = global_path_.poses[j + 1].pose.position;
            double ds = hypot(p2.x - p1.x, p2.y - p1.y);
            acc += ds;
            if (acc >= lookahead_dist_)
            {
                return p2;
            }
        }
        // 不足前视距离则返回终点
        return global_path_.poses.back().pose.position;
    }

    // void loadParameters() {
    //     // 速度参数
    //     nh_.param("max_vel_x", config_.max_v, 1.0);
    //     nh_.param("min_vel_x", config_.min_v, 0.0);
    //     nh_.param("max_vel_theta", config_.max_w, 1.0);
    //     nh_.param("min_vel_theta", config_.min_w, -1.0);
    //     // 加速度参数
    //     nh_.param("acc_lim_x", config_.max_accel_v, 0.6);
    //     nh_.param("acc_lim_theta", config_.max_accel_w, 0.8);
    //     // 机器人参数
    //     nh_.param("robot_radius", config_.robot_radius, 0.5);
    //     nh_.param("robot_width", config_.robot_width, 0.9);
    //     nh_.param("robot_length", config_.robot_length, 1.8);
    //     // DWA参数
    //     nh_.param("sim_time", config_.predict_time, 3.0);
    //     nh_.param("v_resolution", config_.v_resolution, 0.1);
    //     nh_.param("w_resolution", config_.w_resolution, 0.05);
    //     nh_.param("dt", config_.dt, 0.1);
    //     // 代价函数权重
    //     nh_.param("to_goal_cost_gain", config_.to_goal_cost_gain, 1.0);
    //     nh_.param("speed_cost_gain", config_.speed_cost_gain, 0.1);
    //     nh_.param("obstacle_cost_gain", config_.obstacle_cost_gain, 2.0);
    //     nh_.param("path_cost_gain", config_.path_cost_gain, 1.0);
    //     // 安全参数
    //     nh_.param("safe_dist", config_.safe_dist, 0.2);
    //     nh_.param("goal_tolerance", config_.goal_tolerance, 0.2);
    //     ROS_INFO("DWA parameters loaded");
    // }

    void mapRequestTimerCallback(const ros::TimerEvent &)
    {
        ROS_INFO("Timer triggered, requesting map...");
        requestMap();
    }

    void requestMap()
    {
        std::string map_service_name = "/static_map";
        nh_.param("map_service", map_service_name, std::string("/static_map"));

        ROS_INFO("Waiting for map service: %s", map_service_name.c_str());

        if (!map_client_.waitForExistence(ros::Duration(10.0)))
        {
            ROS_ERROR("Map service %s not available after 10 seconds", map_service_name.c_str());
            return;
        }

        nav_msgs::GetMap srv;
        if (map_client_.call(srv))
        {
            global_map_ = srv.response.map;
            map_received_ = true;

            ROS_INFO("Map received successfully!");
            ROS_INFO("Map info - Width: %d, Height: %d, Resolution: %.3f",
                     global_map_.info.width,
                     global_map_.info.height,
                     global_map_.info.resolution);

            updateLocalMapWithObstacles();
        }
        else
        {
            ROS_ERROR("Failed to call map service %s", map_service_name.c_str());
        }
    }

    void initCustomObstacles()
    {
        // 示例：添加一些自定义圆形障碍物
        custom_obstacles_.clear();
        custom_obstacles_.emplace_back(-10.0, -1, 0.6, 0.7, 0);
        custom_obstacles_.emplace_back(-18.0, 0.5, 0.6, 0.7, 0);
        publishCustomObstacles();
    }

    // void odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
    // {
    //     current_pose_.x = msg->pose.pose.position.x;
    //     current_pose_.y = msg->pose.pose.position.y;

    //     tf2::Quaternion q;
    //     tf2::fromMsg(msg->pose.pose.orientation, q);
    //     tf2::Matrix3x3 m(q);
    //     double roll, pitch;
    //     m.getRPY(roll, pitch, current_pose_.theta);

    //     current_vel_.v = 0.3;
    //     current_vel_.w = 0;

    //     odom_received_ = true;
    // }

    void odometryGetCallBack_slam(const nav_msgs::Odometry::ConstPtr &msg)
    {

        // current_pose_.x = msg->pose.pose.position.x;
        // current_pose_.y = msg->pose.pose.position.y;

        tf2::Quaternion q;
        tf2::fromMsg(msg->pose.pose.orientation, q);
        tf2::Matrix3x3 m(q);
        double roll, pitch;
        m.getRPY(roll, pitch, current_pose_.theta);

        current_pose_.x = msg->pose.pose.position.x - cos(current_pose_.theta) * (0.9); // 纵向
        current_pose_.y = msg->pose.pose.position.y - sin(current_pose_.theta) * (0.9); // 纵向

        current_vel_.v = 0.3;
        current_vel_.w = 0;

        odom_received_ = true;
    }

    void pathCallback(const nav_msgs::Path::ConstPtr &msg)
    {
        global_path_ = *msg;
        path_received_ = true;
        ROS_INFO("Global path received with %zu waypoints", global_path_.poses.size());
    }

    void customObstaclesCallback(const std_msgs::Float64MultiArray::ConstPtr &msg)
    {
        custom_obstacles_.clear();
        for (size_t i = 0; i + 4 < msg->data.size(); i += 5)
        {
            custom_obstacles_.emplace_back(msg->data[i], msg->data[i + 1], msg->data[i + 2], msg->data[i + 3], msg->data[i + 4]);
        }
        ROS_INFO("Received %zu custom obstacles", custom_obstacles_.size());
        // updateLocalMapWithObstacles();
        publishCustomObstacles();
    }

    void updateLocalMapWithObstacles()
    {
        if (!map_received_)
            return;
        local_map_ = global_map_;
        for (const auto &obs : custom_obstacles_)
        {
            double dx = obs.x - current_pose_.x, dy = obs.y - current_pose_.y;
            if (std::hypot(dx, dy) < obstacle_map_range_)
            {
                addRectObstacleToMap(obs.x, obs.y, obs.length, obs.width, obs.theta, config_.safe_dist);
            }
        }
        cropLocalMapToCircularROI(current_pose_.x, current_pose_.y, roi_radius_);
        local_map_.header.stamp = ros::Time::now();
        local_map_.header.frame_id = "map";
        local_map_pub_.publish(local_map_);
    }

    // 生成矩形顶点
    std::vector<std::pair<double, double>> getRectCorners(double x, double y, double length, double width, double theta)
    {
        double hl = length / 2, hw = width / 2;
        double cos_t = cos(theta), sin_t = sin(theta);
        std::vector<std::pair<double, double>> corners;
        for (int dx : {-1, 1})
        {
            for (int dy : {-1, 1})
            {
                double local_x = dx * hl, local_y = dy * hw;
                double wx = x + local_x * cos_t - local_y * sin_t;
                double wy = y + local_x * sin_t + local_y * cos_t;
                corners.emplace_back(wx, wy);
            }
        }
        return corners;
    }

    // 判断两个凸四边形是否分离
    bool polygonsSeparated(const std::vector<std::pair<double, double>> &poly1, const std::vector<std::pair<double, double>> &poly2)
    {
        auto getAxes = [](const std::vector<std::pair<double, double>> &poly)
        {
            std::vector<std::pair<double, double>> axes;
            for (size_t i = 0; i < poly.size(); ++i)
            {
                double dx = poly[(i + 1) % poly.size()].first - poly[i].first;
                double dy = poly[(i + 1) % poly.size()].second - poly[i].second;
                // 法向量
                axes.emplace_back(-dy, dx);
            }
            return axes;
        };
        auto axes1 = getAxes(poly1), axes2 = getAxes(poly2);
        for (auto axis_group : {axes1, axes2})
        {
            for (auto axis : axis_group)
            {
                double min1 = 1e9, max1 = -1e9, min2 = 1e9, max2 = -1e9;
                for (auto pt : poly1)
                {
                    double proj = pt.first * axis.first + pt.second * axis.second;
                    min1 = std::min(min1, proj);
                    max1 = std::max(max1, proj);
                }
                for (auto pt : poly2)
                {
                    double proj = pt.first * axis.first + pt.second * axis.second;
                    min2 = std::min(min2, proj);
                    max2 = std::max(max2, proj);
                }
                if (max1 < min2 || max2 < min1)
                    return true; // 存在分离轴
            }
        }
        return false; // 没有分离轴就重叠
    }

    void addRectObstacleToMap(double cx, double cy, double length, double width, double theta, double inflate)
    {
        double res = local_map_.info.resolution;
        int map_width = local_map_.info.width, map_height = local_map_.info.height;
        double ox = local_map_.info.origin.position.x, oy = local_map_.info.origin.position.y;
        // 1. 先扩大障碍物尺寸
        double half_l = length / 2 + inflate / 2, half_w = width / 2 + inflate / 2;

        // 2. 计算覆盖的栅格区域（粗略外接框）
        int min_x = std::max(0, int((cx - half_l - ox) / res));
        int max_x = std::min(map_width - 1, int((cx + half_l - ox) / res));
        int min_y = std::max(0, int((cy - half_w - oy) / res));
        int max_y = std::min(map_height - 1, int((cy + half_w - oy) / res));

        double cos_t = cos(theta), sin_t = sin(theta);

        for (int x = min_x; x <= max_x; ++x)
        {
            for (int y = min_y; y <= max_y; ++y)
            {
                double wx = ox + x * res;
                double wy = oy + y * res;

                // 坐标系变换到障碍物本地
                double dx = wx - cx, dy = wy - cy;
                double local_x = dx * cos_t + dy * sin_t;
                double local_y = -dx * sin_t + dy * cos_t;

                if (fabs(local_x) <= half_l && fabs(local_y) <= half_w)
                {
                    int idx = y * map_width + x;
                    if (idx >= 0 && idx < (int)local_map_.data.size())
                    {
                        local_map_.data[idx] = 100; // 直接填满为障碍
                    }
                }
            }
        }
    }

    void cropLocalMapToCircularROI(double cx, double cy, double radius)
    {
        // ROI窗口形状为圆形，超出边界保护
        if (!map_received_)
            return;
        double res = local_map_.info.resolution;
        int width = local_map_.info.width, height = local_map_.info.height;
        double ox = local_map_.info.origin.position.x;
        double oy = local_map_.info.origin.position.y;

        std::vector<int8_t> new_data(local_map_.data.size(), -1);
        for (int y = 0; y < height; ++y)
        {
            for (int x = 0; x < width; ++x)
            {
                double wx = ox + x * res;
                double wy = oy + y * res;
                double dist = std::hypot(wx - cx, wy - cy);
                int idx = y * width + x;
                if (idx < 0 || idx >= (int)local_map_.data.size())
                    continue;
                if (dist <= radius)
                    new_data[idx] = local_map_.data[idx];
            }
        }
        local_map_.data = new_data;
    }

    void publishLocalPath(const std::vector<Pose> &trajectory)
    {
        nav_msgs::Path path_msg;
        path_msg.header.stamp = ros::Time::now();
        path_msg.header.frame_id = "odom";
        for (const auto &pose : trajectory)
        {
            geometry_msgs::PoseStamped p;
            p.header = path_msg.header;
            p.pose.position.x = pose.x;
            p.pose.position.y = pose.y;
            p.pose.position.z = 0.0;
            tf2::Quaternion q;
            q.setRPY(0, 0, pose.theta);
            p.pose.orientation = tf2::toMsg(q);
            path_msg.poses.push_back(p);
        }
        local_path_pub_.publish(path_msg);
    }

    // ==================== DWA 核心算法 ====================

    // std::vector<Velocity> generateDynamicWindow(const Velocity& current_vel) {
    //     std::vector<Velocity> velocities;

    //     // 计算动态窗口边界
    //     double min_v = std::max(config_.min_v, current_vel.v - config_.max_accel_v * config_.dt);
    //     double max_v = std::min(config_.max_v, current_vel.v + config_.max_accel_v * config_.dt);
    //     double min_w = std::max(config_.min_w, current_vel.w - config_.max_accel_w * config_.dt);
    //     double max_w = std::min(config_.max_w, current_vel.w + config_.max_accel_w * config_.dt);

    //     // 生成速度候选
    //     for (double v = min_v; v <= max_v + 1e-6; v += config_.v_resolution) {
    //         for (double w = min_w; w <= max_w + 1e-6; w += config_.w_resolution) {
    //             velocities.emplace_back(v, w);
    //         }
    //     }

    //     return velocities;
    // }

    std::vector<Velocity> generateDynamicWindow(const Velocity &current_vel)
    {
        std::vector<Velocity> velocities;

        // 计算动态窗口边界
        double min_v = std::max(config_.min_v, current_vel.v - config_.max_accel_v * config_.dt);
        double max_v = std::min(config_.max_v, current_vel.v + config_.max_accel_v * config_.dt);
        double min_w = std::max(config_.min_w, current_vel.w - config_.max_accel_w * config_.dt);
        double max_w = std::min(config_.max_w, current_vel.w + config_.max_accel_w * config_.dt);

        // 生成速度候选
        for (double v = 0.3; v <= 0.3 + 1e-6; v += 0.1)
        {
            for (double w = min_w; w <= max_w + 1e-6; w += config_.w_resolution)
            {
                velocities.emplace_back(v, w);
            }
        }

        return velocities;
    }

    std::vector<Pose> predictTrajectory(const Pose &start_pose, const Velocity &vel)
    {
        std::vector<Pose> trajectory;
        Pose current = start_pose;

        double time = 0.0;
        while (time <= config_.predict_time)
        {
            trajectory.push_back(current);

            // 运动学模型：差分驱动机器人
            current.x += vel.v * cos(current.theta) * config_.dt;
            current.y += vel.v * sin(current.theta) * config_.dt;
            current.theta += vel.w * config_.dt;

            // 角度归一化
            while (current.theta > M_PI)
                current.theta -= 2 * M_PI;
            while (current.theta < -M_PI)
                current.theta += 2 * M_PI;

            time += config_.dt;
        }

        return trajectory;
    }

    bool isTrajectoryCollisionFree(const std::vector<Pose> &trajectory)
    {
        for (const auto &pose : trajectory)
        {
            // 检查与地图障碍物的碰撞
            if (isCollisionWithMap(pose))
            {
                return false;
            }

            // // 检查与自定义障碍物的碰撞
            // if (isCollisionWithCustomObstacles(pose)) {
            //     return false;
            // }
        }
        return true;
    }

    // 先保证你已有 getRobotFootprint(const Pose&) 返回 4 个角点世界坐标

    bool isCollisionWithMap(const Pose &pose)
    {
        if (!map_received_)
            return false;

        // 获取机器人矩形底盘四个角
        auto footprint = getRobotFootprint(pose);

        double resolution = local_map_.info.resolution;
        int width = local_map_.info.width;
        int height = local_map_.info.height;
        double ox = local_map_.info.origin.position.x;
        double oy = local_map_.info.origin.position.y;

        for (const auto &pt : footprint)
        {
            // 角点映射到栅格
            int gx = int((pt.first - ox) / resolution);
            int gy = int((pt.second - oy) / resolution);

            // 越界当成碰撞
            if (gx < 0 || gx >= width || gy < 0 || gy >= height)
            {
                return true;
            }
            int idx = gy * width + gx;
            if (local_map_.data[idx] > 50)
            {
                // 该格被标为障碍
                return true;
            }
        }
        return false;
    }

    bool isCollisionWithCustomObstacles(const Pose &pose)
    {
        auto car = getRectCorners(pose.x, pose.y, config_.robot_length, config_.robot_width, pose.theta);
        for (const auto &obs : custom_obstacles_)
        {
            auto obs_rect = getRectCorners(obs.x, obs.y, obs.length + 2 * config_.safe_dist, obs.width + 2 * config_.safe_dist, obs.theta);
            if (!polygonsSeparated(car, obs_rect)) // 没有分离轴就是碰撞
                return true;
        }
        return false;
    }

    std::vector<std::pair<double, double>> getRobotFootprint(const Pose &pose)
    {
        double L = 1.8, W = 0.9; // 车长和车宽
        double cos_t = cos(pose.theta), sin_t = sin(pose.theta);
        // 底盘中心在 pose.x,pose.y，四角相对偏移：
        std::vector<std::pair<double, double>> pts = {
            {L / 2, W / 2}, {L / 2, -W / 2}, {-L / 2, -W / 2}, {-L / 2, W / 2}};
        std::vector<std::pair<double, double>> world;
        for (auto &p : pts)
        {
            double xw = pose.x + p.first * cos_t - p.second * sin_t;
            double yw = pose.y + p.first * sin_t + p.second * cos_t;
            world.emplace_back(xw, yw);
        }
        return world;
    }

    TrajectoryScore evaluateTrajectory(const std::vector<Pose> &trajectory, const Velocity &vel)
    {
        TrajectoryScore score;

        if (trajectory.empty())
        {
            score.total_cost = std::numeric_limits<double>::max();
            return score;
        }

        // // 1. 目标代价 - 轨迹终点到目标的距离
        // score.to_goal_cost = calcToGoalCost(trajectory);
        geometry_msgs::Point lookahead = computeLookaheadPoint();
        const Pose &end = trajectory.back();
        double dx = lookahead.x - end.x, dy = lookahead.y - end.y;
        score.to_goal_cost = std::hypot(dx, dy);

        // 2. 速度代价 - 鼓励较高的前进速度
        score.speed_cost = calcSpeedCost(vel);

        // 3. 障碍物代价 - 惩罚接近障碍物的轨迹
        score.obstacle_cost = calcObstacleCost(trajectory);

        // 4. 路径代价 - 鼓励跟随全局路径
        score.path_cost = calcPathCost(trajectory);

        // 计算总代价
        score.total_cost = config_.to_goal_cost_gain * score.to_goal_cost +
                           config_.speed_cost_gain * score.speed_cost +
                           config_.obstacle_cost_gain * score.obstacle_cost +
                           config_.path_cost_gain * score.path_cost;

        return score;
    }

    double calcToGoalCost(const std::vector<Pose> &trajectory)
    {
        if (global_path_.poses.empty() || trajectory.empty())
        {
            return std::numeric_limits<double>::max();
        }

        // 获取轨迹终点
        const Pose &end_pose = trajectory.back();

        // 找到全局路径的目标点（最后一个点）
        const auto &goal = global_path_.poses.back().pose.position;

        double dx = goal.x - end_pose.x;
        double dy = goal.y - end_pose.y;
        return sqrt(dx * dx + dy * dy);
    }

    double calcSpeedCost(const Velocity &vel)
    {
        // 鼓励更高的前进速度
        return config_.max_v - vel.v;
    }

    double calcObstacleCost(const std::vector<Pose> &trajectory)
    {
        double min_dist = std::numeric_limits<double>::max();

        for (const auto &pose : trajectory)
        {
            // 计算到地图障碍物的最小距离
            double map_dist = distanceToMapObstacle(pose);
            min_dist = std::min(min_dist, map_dist);

            // 计算到自定义障碍物的最小距离
            double custom_dist = distanceToCustomObstacles(pose);
            min_dist = std::min(min_dist, custom_dist);
        }

        // 如果距离太近，返回高代价
        if (min_dist < config_.safe_dist)
        {
            return std::numeric_limits<double>::max();
        }

        // 距离越近代价越高
        return 1.0 / (min_dist + 0.1);
    }

    double calcPathCost(const std::vector<Pose> &trajectory)
    {
        if (global_path_.poses.empty() || trajectory.empty())
        {
            return 0.0;
        }

        double total_cost = 0.0;

        for (const auto &traj_pose : trajectory)
        {
            double min_dist = std::numeric_limits<double>::max();

            // 找到轨迹点到全局路径的最小距离
            for (const auto &path_pose : global_path_.poses)
            {
                double dx = path_pose.pose.position.x - traj_pose.x;
                double dy = path_pose.pose.position.y - traj_pose.y;
                double dist = sqrt(dx * dx + dy * dy);
                min_dist = std::min(min_dist, dist);
            }

            total_cost += min_dist;
        }

        return total_cost / trajectory.size();
    }

    double distanceToMapObstacle(const Pose &pose)
    {
        if (!map_received_)
            return std::numeric_limits<double>::max();

        double resolution = local_map_.info.resolution;
        int map_width = local_map_.info.width;
        int map_height = local_map_.info.height;
        double origin_x = local_map_.info.origin.position.x;
        double origin_y = local_map_.info.origin.position.y;

        int grid_x = (int)((pose.x - origin_x) / resolution);
        int grid_y = (int)((pose.y - origin_y) / resolution);

        if (grid_x < 0 || grid_x >= map_width || grid_y < 0 || grid_y >= map_height)
        {
            return 0.0; // 超出边界
        }

        // 搜索最近的障碍物
        double min_dist = std::numeric_limits<double>::max();
        int search_radius = (int)(3.0 / resolution); // 搜索3米半径

        for (int dx = -search_radius; dx <= search_radius; ++dx)
        {
            for (int dy = -search_radius; dy <= search_radius; ++dy)
            {
                int check_x = grid_x + dx;
                int check_y = grid_y + dy;

                if (check_x >= 0 && check_x < map_width && check_y >= 0 && check_y < map_height)
                {
                    int index = check_y * map_width + check_x;
                    if (index >= 0 && index < local_map_.data.size())
                    {
                        if (local_map_.data[index] > 50)
                        {
                            double dist = sqrt(dx * dx + dy * dy) * resolution;
                            min_dist = std::min(min_dist, dist);
                        }
                    }
                }
            }
        }

        return min_dist;
    }

    double distanceToCustomObstacles(const Pose &pose)
    {
        double min_dist = std::numeric_limits<double>::max();

        for (const auto &obstacle : custom_obstacles_)
        {
            double dx = pose.x - obstacle.x;
            double dy = pose.y - obstacle.y;
            double l = obstacle.length / 2;
            double r = obstacle.width / 2;
            double obstacle_r = sqrt(l * l + r * r);
            double dist = sqrt(dx * dx + dy * dy) - obstacle_r;
            min_dist = std::min(min_dist, dist);
        }

        return min_dist;
    }

    Velocity selectBestVelocity()
    {
        if (!odom_received_)
        {
            return Velocity(0.0, 0.0);
        }

        // 生成动态窗口
        std::vector<Velocity> velocities = generateDynamicWindow(current_vel_);

        std::vector<Velocity> valid_velocities;
        std::vector<TrajectoryScore> scores;
        all_trajectories_.clear();

        // 评估每个速度候选
        for (const auto &vel : velocities)
        {
            std::vector<Pose> trajectory = predictTrajectory(current_pose_, vel);

            // 检查轨迹是否无碰撞
            if (isTrajectoryCollisionFree(trajectory))
            {
                TrajectoryScore score = evaluateTrajectory(trajectory, vel);

                valid_velocities.push_back(vel);
                scores.push_back(score);
                all_trajectories_.push_back(trajectory);
            }
        }

        if (valid_velocities.empty())
        {
            ROS_WARN("No valid velocities found! Emergency stop.");
            return Velocity(0.0, 0.0);
        }

        // 选择代价最小的速度
        auto min_it = std::min_element(scores.begin(), scores.end(),
                                       [](const TrajectoryScore &a, const TrajectoryScore &b)
                                       {
                                           return a.total_cost < b.total_cost;
                                       });

        int best_index = std::distance(scores.begin(), min_it);
        trajectory_scores_ = scores;

        ROS_DEBUG("Best velocity: v=%.2f, w=%.2f, cost=%.3f",
                  valid_velocities[best_index].v,
                  valid_velocities[best_index].w,
                  min_it->total_cost);

        return valid_velocities[best_index];
    }

    void controlLoop(const ros::TimerEvent &)
    {
        // initCustomObstacles();
        ros::param::get("shifoubizhang", ls_avoidance_);
        // std::cout << "////////is_avoidance///" << ls_avoidance_ << std::endl; // lzw_1114注释
        // std::cout << "////////is_ready_to_avoidance///" << ls_ready_to_avoidance_ << std::endl; // lzw_1114注释
        if (ls_avoidance_ && ls_ready_to_avoidance_)
        {
            ROS_INFO_THROTTLE(2, "///////start dwa, kaishibizhang/////////// ");
            if (!odom_received_ || !path_received_)
            {

                // 只有两者都准备好才做 DWA，否则发 0
                geometry_msgs::Twist stop;
                stop.linear.x = 0;
                stop.angular.z = 0;
                cmd_vel_pub_.publish(stop);
                return;
            }
            // 更新局部地图
            //  updateLocalMapWithObstacles();

            // 选择最佳速度
            Velocity best_vel = selectBestVelocity();

            // 检查是否到达目标
            if (isGoalReached())
            {
                best_vel = Velocity(0.0, 0.0);
                ROS_INFO_THROTTLE(2.0, "Goal reached!");
            }
            //////////////////
            if (best_vel.v > 1e-3 || std::abs(best_vel.w) > 1e-3)
            {
                // DWA找到了可行轨迹，发布最优轨迹为局部路径
                if (all_trajectories_.size() > 0 && trajectory_scores_.size() > 0)
                {
                    int best_index = std::min_element(trajectory_scores_.begin(), trajectory_scores_.end(),
                                                      [](const TrajectoryScore &a, const TrajectoryScore &b)
                                                      {
                                                          return a.total_cost < b.total_cost;
                                                      }) -
                                     trajectory_scores_.begin();
                    publishLocalPath(all_trajectories_[best_index]);
                    ros::param::set("is_local_path_available", true);
                }
                else
                {
                    // 没有可行轨迹
                    publishLocalPath({});
                    ros::param::set("is_local_path_available", false);
                }
            }
            else
            {
                // DWA未找到可行轨迹或机器人已停止，发布空路径
                publishLocalPath({});
                ros::param::set("is_local_path_available", false);
            }
            // 发布控制命令
            geometry_msgs::Twist cmd_vel;
            cmd_vel.linear.x = best_vel.v;
            cmd_vel.angular.z = best_vel.w;
            cmd_vel_pub_.publish(cmd_vel);

            // 发布可视化信息
            publishTrajectories();
            publishRobotMarker(); // 新增：在 RViz 里画出机器人

            ROS_INFO_THROTTLE(1.0, "Cmd_vel: linear=%.2f, angular=%.2f",
                              best_vel.v, best_vel.w);
        }
        else
        {
            ROS_INFO_THROTTLE(2, "///////no avoidance!!!/////////// ");
        }
    }

    bool isGoalReached()
    {
        if (!path_received_ || global_path_.poses.empty())
        {
            return false;
        }

        const auto &goal = global_path_.poses.back().pose.position;
        double dx = goal.x - current_pose_.x;
        double dy = goal.y - current_pose_.y;
        double dist = sqrt(dx * dx + dy * dy);

        return dist < config_.goal_tolerance;
    }

    void publishTrajectories()
    {
        visualization_msgs::MarkerArray marker_array;

        for (size_t i = 0; i < all_trajectories_.size(); ++i)
        {
            visualization_msgs::Marker marker;
            marker.header.frame_id = "odom";
            marker.header.stamp = ros::Time::now();
            marker.ns = "trajectories";
            marker.id = i;
            marker.type = visualization_msgs::Marker::LINE_STRIP;
            marker.action = visualization_msgs::Marker::ADD;

            marker.scale.x = 0.02;

            // 根据代价设置颜色
            if (i < trajectory_scores_.size())
            {
                double normalized_cost = std::min(1.0, trajectory_scores_[i].total_cost / 10.0);
                marker.color.r = normalized_cost;
                marker.color.g = 1.0 - normalized_cost;
                marker.color.b = 0.0;
                marker.color.a = 0.7;
            }
            else
            {
                marker.color.r = 1.0;
                marker.color.g = 0.0;
                marker.color.b = 0.0;
                marker.color.a = 0.5;
            }

            for (const auto &pose : all_trajectories_[i])
            {
                geometry_msgs::Point point;
                point.x = pose.x;
                point.y = pose.y;
                point.z = 0.0;
                marker.points.push_back(point);
            }

            marker_array.markers.push_back(marker);
        }

        trajectory_pub_.publish(marker_array);
    }

    void publishCustomObstacles()
    {
        visualization_msgs::MarkerArray marker_array;
        for (size_t i = 0; i < custom_obstacles_.size(); ++i)
        {
            visualization_msgs::Marker marker;
            marker.header.frame_id = "odom";
            marker.header.stamp = ros::Time::now();
            marker.ns = "custom_obstacles";
            marker.id = i;
            marker.type = visualization_msgs::Marker::CUBE;
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose.position.x = custom_obstacles_[i].x;
            marker.pose.position.y = custom_obstacles_[i].y;
            marker.pose.position.z = 0.05;
            tf2::Quaternion q;
            q.setRPY(0, 0, custom_obstacles_[i].theta);
            marker.pose.orientation = tf2::toMsg(q);
            marker.scale.x = custom_obstacles_[i].length;
            marker.scale.y = custom_obstacles_[i].width;
            marker.scale.z = 0.1;
            marker.color.r = 1.0;
            marker.color.g = 0.0;
            marker.color.b = 0.0;
            marker.color.a = 0.8;
            marker_array.markers.push_back(marker);
        }
        obstacles_pub_.publish(marker_array);
    }

    void publishRobotMarker()
    {
        visualization_msgs::Marker m;
        m.header.frame_id = "odom";
        m.header.stamp = ros::Time::now();
        m.ns = "robot";
        m.id = 0;
        m.type = visualization_msgs::Marker::CUBE;
        m.action = visualization_msgs::Marker::ADD;

        m.pose.position.x = current_pose_.x;
        m.pose.position.y = current_pose_.y;
        m.pose.position.z = 0.05;
        tf2::Quaternion q;
        q.setRPY(0, 0, current_pose_.theta);
        m.pose.orientation = tf2::toMsg(q);

        m.scale.x = 1.8; // 车长
        m.scale.y = 0.9; // 车宽
        m.scale.z = 0.1;

        m.color.r = 0.0;
        m.color.g = 0.5;
        m.color.b = 1.0;
        m.color.a = 0.8;
        car_pub_.publish(m); // 或者开一个专门的 publisher
    }

    // 公共接口
    void addCustomObstacle(double x, double y, double l, double d, double w)
    {
        custom_obstacles_.emplace_back(x, y, l, d, w);
        updateLocalMapWithObstacles();
        publishCustomObstacles();
        ROS_INFO("Added custom obstacle at (%.2f, %.2f) with radius %.2f", x, y, w);
    }

    void clearCustomObstacles()
    {
        custom_obstacles_.clear();
        updateLocalMapWithObstacles();
        publishCustomObstacles();
        ROS_INFO("Cleared all custom obstacles");
    }

    void run()
    {
        ROS_INFO("DWA Local Planner running...");
        ros::spin();
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "dwa_local_planner");

    DWALocalPlanner planner;
    planner.run();

    return 0;
}