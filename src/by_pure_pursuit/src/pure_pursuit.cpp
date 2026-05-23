#include "include/headfile.h"
#include <path_planning_msgs/PathPoint.h>
#include <path_planning_msgs/Path.h>
#include <std_msgs/Bool.h>
bool GPS = -1;
bool ODOM = -1;
bool SLAM = -1;

float turn_limit = 0.31;
int slow_flag = 0;
class pure_pursuit
{
public:
    int task = 0;
    bool first_flag = 0;
    int stop_mode = 0;
    int angle_pid_flag = 0;
    int stop_lidar_mode = 0;
    int stop_camera_mode = 0;
    bool ls_avoidance = 0;
    ros::Publisher pub_control_cmd;
    ros::Publisher marker_track_point_rviz;
    ros::Publisher Ready_to_obstacle_pub;

    vector<Point> track_road;
    vector<Point> multi_path_road;

    State vehicleState;

    void node_start(int argc, char *argv[]);
    void get_control_callback(const by_global_path_planning::Path msg);
    void track_follow_process(const State &s, vector<Point> track_road);
    void track_follow_process_obs(const State &s, vector<Point> track_road);
    void odometryGetCallBack_GPS(const geometry_msgs::PoseStamped::ConstPtr odometry_msg);
    void odometryGetCallBack_slam(const nav_msgs::Odometry::ConstPtr odometry_msg);
    void odometryGetCallBack(const nav_msgs::Odometry::ConstPtr odometry_msg);
    void multi_path_callback(const path_planning_msgs::Path msg);
    void avoidance_callback(const std_msgs::Bool msg);
    void stop_deal();
};
void pure_pursuit::stop_deal()
{
    // 进入停车控制模式
    //  cout<<"y"<<track_road[track_road.size()-1].y<<endl; // lzw_1118注释掉
    //  cout<<"x"<<track_road[track_road.size()-1].x<<endl; // lzw_1118注释掉
    float dis = sqrt(pow(vehicleState.x - track_road[track_road.size() - 1].x, 2) + pow(vehicleState.y - track_road[track_road.size() - 1].y, 2));
    float wheelAngle = atan2(track_road[track_road.size() - 1].y - vehicleState.y, track_road[track_road.size() - 1].x - vehicleState.x) - vehicleState.yaw; // 误差角度
    geometry_msgs::Twist cmd_vel;
    if ((abs(wheelAngle) * 53.5 > 5 && dis < 0.6) || angle_pid_flag)
    { // 走过终点后，进一次就必进 // 0929：原先是if((abs(wheelAngle)*53.5>5 && dis<0.6) || angle_pid_flag)
        angle_pid_flag = 1;
        if (abs(track_road[track_road.size() - 1].theta - vehicleState.yaw) < 0.1)
        { // 0929：原先是if(abs(track_road[track_road.size()-1].theta-vehicleState.yaw)<0.05 )
            // 普通寻迹逻辑
            cout << "-------------------------------stop_over-----------------------" << endl;
            for (int i = 0; i < 20; i++)
            {
                cmd_vel.linear.x = 0;
                cmd_vel.linear.y = 0;
                cmd_vel.angular.z = 0;
                pub_control_cmd.publish(cmd_vel);
            }
            first_flag = 0;
            stop_mode = 0;
            angle_pid_flag = 0;
            if (task == 9)
            {
                ros::param::set("renwu_finish", 9);
                ros::param::set("work_state", 1);
            }
            else if (task == 1)
            {
                ros::param::set("renwu_finish", 1);
                ros::param::set("work_state", 1);
            }
            else if (task == 11)
            {
                ros::param::set("renwu_finish", 11);
                ros::param::set("work_state", 1);
            }
        }
        else
        { // 角度调整
            cout << "stop_222" << endl;
            cmd_vel.linear.x = 0;
            cmd_vel.linear.y = 0;
            float err_theta = track_road[track_road.size() - 1].theta - vehicleState.yaw;
            if (err_theta > 3.14)
            {
                err_theta = err_theta - 6.28;
            }
            if (err_theta < -3.14)
            {
                err_theta = err_theta + 6.28;
            }

            cmd_vel.angular.z = -PID_Realize1(&stop3_angle_PID, stop3_angle_pid, err_theta * 100, 0);

            range_precote(cmd_vel.angular.z, 0.1, -0.1);
            pub_control_cmd.publish(cmd_vel);
        }
    }
    else
    { // 正常寻
        cout << "stop_111" << endl;
        slow_flag = 1;
        track_follow_process(this->vehicleState, track_road);
    }
}
void pure_pursuit::odometryGetCallBack_slam(const nav_msgs::Odometry::ConstPtr odometry_msg)
{
    ros::Time time_now = ros::Time::now();
    if (SLAM)
    {
        ros::Rate loop_rate(10);
        double raw, pitch, theta;
        tf::Quaternion q;
        tf::quaternionMsgToTF(odometry_msg->pose.pose.orientation, q);
        tf::Matrix3x3(q).getRPY(raw, pitch, theta);
        this->vehicleState.yaw = theta;
        this->vehicleState.x = odometry_msg->pose.pose.position.x - cos(theta) * (0.9); // 纵向
        this->vehicleState.y = odometry_msg->pose.pose.position.y - sin(theta) * (0.9); // 纵向

        // this->vehicle_pose.theta = -theta;
        // this->vehicle_pose.x = odometry_msg->pose.pose.position.x;
        // this->vehicle_pose.y = -odometry_msg->pose.pose.position.y;

        loop_rate.sleep();
        if (first_flag)
        {
            // track_follow_process(this->vehicleState, track_road);
            if (sqrt(pow(vehicleState.x - track_road[track_road.size() - 1].x, 2) + pow(vehicleState.y - track_road[track_road.size() - 1].y, 2)) < 2)
            {
                stop_mode = 1;
            }
            if (stop_mode == 0)
            {
                ros::param::set("work_state", 2);
                ros::param::get("shifoubizhang", this->ls_avoidance);
                if (ls_avoidance)
                {
                    ///////////////////////////// 以下：lzw改：加入DWA切换自旋 //////////////////////////////////////////////////////////
                    double wheelAngle = 0;
                    geometry_msgs::Twist cmd_vel;
                    int index = 1; // latisi::get_goal_index(s, path); // 在车当前最近点加上前视距离的道路的坐标点
                    float dis_min = 100000;
                    for (int i = 0; i < track_road.size(); i++)
                    {
                        float dis = pow(vehicleState.x - track_road[i].x, 2) + pow(vehicleState.y - track_road[i].y, 2);
                        if (dis < dis_min)
                        {
                            dis_min = dis;
                            index = i;
                        }
                    }
                    range_precote(index, track_road.size() - 3, 0); // 防止index溢出
                    Point goal = track_road[index + 2];
                    double alpha = atan2(goal.y - vehicleState.y, goal.x - vehicleState.x) - vehicleState.yaw;
                    if (alpha > 3.14)
                    {
                        alpha = alpha - 6.28;
                    }
                    if (alpha < -3.14)
                    {
                        alpha = alpha + 6.28;
                    }
                    // cout<<"zhengchang"<<endl;
                    // cout<<"atan2( goal.y - s.y,goal.x - s.x) "<<atan2( goal.y - s.y,goal.x - s.x)<<endl;
                    // cout<<"s.yaw "<<s.yaw<<endl;
                    wheelAngle = 0.625 * alpha;
                    // cout<<"wheelAngle"<<wheelAngle<<endl;
                    bool is_local_path_available = true;
                    ros::param::get("is_local_path_available", is_local_path_available); // lzw改：新增DWA-自旋的参数服务器通信
                    if (abs(wheelAngle) > 1.74 && false)
                    {
                        std_msgs::Bool msg;
                        msg.data = false;
                        Ready_to_obstacle_pub.publish(msg);
                        cmd_vel.angular.z = wheelAngle;
                        cmd_vel.linear.x = 0;
                        this->pub_control_cmd.publish(cmd_vel);
                        rviz_track_point(marker_track_point_rviz, goal);
                        cout << "//////转角偏差过大导致从DWA切换为自旋///////////" << std::endl;
                    }
                    else if (!is_local_path_available && false) // lzw改：
                    {
                        std_msgs::Bool msg;
                        msg.data = false;
                        Ready_to_obstacle_pub.publish(msg);
                        cmd_vel.angular.z = wheelAngle;
                        // cmd_vel.angular.z = 0.2; // lzw改：无DWA局部路径的恢复行为：以0.1角速度进行自旋，自旋到某一位置时一旦DWA规划出局部路径，那么就退出恢复行为、重新DWA
                        cmd_vel.linear.x = 0;
                        this->pub_control_cmd.publish(cmd_vel);
                        rviz_track_point(marker_track_point_rviz, goal);
                        cout << "//////恢复行为：从DWA切换为自旋///////////" << std::endl;
                        ///////////////////////////// 以上：lzw改：加入DWA切换自旋 //////////////////////////////////////////////////////////
                    }
                    else
                    {
                        std_msgs::Bool msg;
                        msg.data = true;
                        Ready_to_obstacle_pub.publish(msg);
                        ROS_INFO_THROTTLE(3, "//////kaishibizhang--using DWA///////////");
                    }
                }
                else
                {
                    std_msgs::Bool msg;
                    msg.data = false;
                    Ready_to_obstacle_pub.publish(msg);
                    ROS_INFO_THROTTLE(2, "////kaishixunji///////////");
                    track_follow_process(this->vehicleState, track_road);
                }
            }
            else
            {
                std_msgs::Bool msg;
                msg.data = false;
                Ready_to_obstacle_pub.publish(msg);
                stop_deal();
            }
        }
    }
    ros::Time time_now_2 = ros::Time::now();
    std::cout << "-----------odometryGetCallBack_slam cost:  " << time_now_2 - time_now << std::endl;
}
void pure_pursuit::odometryGetCallBack(const nav_msgs::Odometry::ConstPtr odometry_msg)
{
    if (ODOM)
    {
        double raw, pitch, theta;
        tf::Quaternion q;
        tf::quaternionMsgToTF(odometry_msg->pose.pose.orientation, q);
        tf::Matrix3x3(q).getRPY(raw, pitch, theta);
        double g_velocity = std::sqrt(std::pow(odometry_msg->twist.twist.linear.x, 2) + std::pow(odometry_msg->twist.twist.linear.y, 2));
        this->vehicleState.x = odometry_msg->pose.pose.position.x;
        this->vehicleState.y = odometry_msg->pose.pose.position.y;
        this->vehicleState.speed = g_velocity;
        this->vehicleState.yaw = theta;
        this->vehicleState.yawrate = odometry_msg->twist.twist.angular.z;
        if (first_flag)
        {
            if (sqrt(pow(vehicleState.x - track_road[track_road.size() - 1].x, 2) + pow(vehicleState.y - track_road[track_road.size() - 1].y, 2)) < 3)
            {
                stop_mode = 1;
            }
            if (stop_mode == 0)
            {
                track_follow_process(this->vehicleState, track_road);
            }
            else
            {
                stop_deal();
            }
        }
    }
}
void pure_pursuit::odometryGetCallBack_GPS(const geometry_msgs::PoseStamped::ConstPtr odometry_msg)
{
    if (GPS)
    {
        ros::Rate loop_rate(10);
        double raw, pitch, theta;
        tf::Quaternion q;
        tf::quaternionMsgToTF(odometry_msg->pose.orientation, q);
        tf::Matrix3x3(q).getRPY(raw, pitch, theta);

        vehicleState.x = -odometry_msg->pose.position.x;
        vehicleState.y = odometry_msg->pose.position.y;
        vehicleState.yaw = -theta;

        if (first_flag)
        {
            if (sqrt(pow(vehicleState.x - track_road[track_road.size() - 1].x, 2) + pow(vehicleState.y - track_road[track_road.size() - 1].y, 2)) < 3)
            {
                stop_mode = 1;
            }
            if (stop_mode == 0)
            {
                track_follow_process(this->vehicleState, track_road);
            }
            else
            {
                stop_deal();
            }
        }
        0.15;
    }
}
void pure_pursuit::track_follow_process(const State &s, vector<Point> track_road)
{
    double wheelAngle = 0;
    geometry_msgs::Twist cmd_vel;
    int index = 1; // latisi::get_goal_index(s, path); // 在车当前最近点加上前视距离的道路的坐标点
    float dis_min = 100000;
    for (int i = 0; i < track_road.size(); i++)
    {
        float dis = pow(vehicleState.x - track_road[i].x, 2) + pow(vehicleState.y - track_road[i].y, 2);
        if (dis < dis_min)
        {
            dis_min = dis;
            index = i;
        }
    }
    range_precote(index, track_road.size() - 3, 0); // 防止index溢出
    Point goal = track_road[index + 2];
    double alpha = atan2(goal.y - s.y, goal.x - s.x) - s.yaw;
    if (alpha > 3.14)
    {
        alpha = alpha - 6.28;
    }
    if (alpha < -3.14)
    {
        alpha = alpha + 6.28;
    }
    // cout << "zhengchang" << endl; // lzw_1118注释掉该行
    // cout << "atan2( goal.y - s.y,goal.x - s.x) " << atan2(goal.y - s.y, goal.x - s.x) << endl; // lzw_1118注释掉该行
    // cout << "s.yaw " << s.yaw << endl; // lzw_1118注释掉该行
    wheelAngle = 0.625 * alpha;
    // cout << "wheelAngle" << wheelAngle << endl; // lzw_1118注释掉该行
    if (abs(wheelAngle) > turn_limit)
    {
        cmd_vel.angular.z = wheelAngle;
        cmd_vel.linear.x = 0;
        this->pub_control_cmd.publish(cmd_vel);
    }
    else if (!slow_flag)
    {
        cmd_vel.angular.z = wheelAngle;
        cmd_vel.linear.x = 0.5;
        this->pub_control_cmd.publish(cmd_vel);
    }
    else
    {
        cmd_vel.angular.z = wheelAngle;
        cmd_vel.linear.x = 0.15;
        this->pub_control_cmd.publish(cmd_vel);
    }
    slow_flag = 0;
    rviz_track_point(marker_track_point_rviz, goal);
}

void pure_pursuit::track_follow_process_obs(const State &s, vector<Point> track_road)
{
    double wheelAngle = 0;
    geometry_msgs::Twist cmd_vel;
    int index = 1; // latisi::get_goal_index(s, path); // 在车当前最近点加上前视距离的道路的坐标点
    float dis_min = 100000;
    for (int i = 0; i < track_road.size(); i++)
    {
        float dis = pow(vehicleState.x - track_road[i].x, 2) + pow(vehicleState.y - track_road[i].y, 2);
        if (dis < dis_min)
        {
            dis_min = dis;
            index = i;
        }
    }
    range_precote(index, track_road.size() - 7, 0); // 防止index溢出
    Point goal = track_road[index + 6];
    double alpha = atan2(goal.y - s.y, goal.x - s.x) - s.yaw;
    if (alpha > 3.14)
    {
        alpha = alpha - 6.28;
    }
    if (alpha < -3.14)
    {
        alpha = alpha + 6.28;
    }
    // cout<<"zhengchang"<<endl;
    // cout<<"atan2( goal.y - s.y,goal.x - s.x) "<<atan2( goal.y - s.y,goal.x - s.x)<<endl;
    // cout<<"s.yaw "<<s.yaw<<endl;
    wheelAngle = 0.5 * alpha;
    // cout<<"wheelAngle"<<wheelAngle<<endl;
    if (abs(wheelAngle) > turn_limit)
    {
        cmd_vel.angular.z = wheelAngle;
        cmd_vel.linear.x = 0;
        this->pub_control_cmd.publish(cmd_vel);
    }
    else if (!slow_flag)
    {
        cmd_vel.angular.z = wheelAngle;
        cmd_vel.linear.x = 0.4;
        this->pub_control_cmd.publish(cmd_vel);
    }
    else
    {
        cmd_vel.angular.z = wheelAngle;
        cmd_vel.linear.x = 0.15;
        this->pub_control_cmd.publish(cmd_vel);
    }
    slow_flag = 0;
    rviz_track_point(marker_track_point_rviz, goal);
}

void pure_pursuit::get_control_callback(const by_global_path_planning::Path msg)
{
    track_road.clear();
    first_flag = 0;
    stop_mode = 0;
    angle_pid_flag = 0;
    for (int i = 0; i < msg.points.size(); i++)
    {
        Point p;
        p.x = msg.points[i].x;
        p.y = msg.points[i].y;
        p.theta = msg.points[i].heading;
        track_road.push_back(p);
        // cout << "sanlou-------------------::::" << p.x << endl; // lzw_1118注释掉该行
        ros::param::set("qqqq", p.x);
    }
    first_flag = 1;
    int sanlou = 1;
    ros::param::set("sanlou", sanlou);
}

void pure_pursuit::multi_path_callback(const path_planning_msgs::Path msg)
{
    this->multi_path_road.clear();
    for (size_t i = 0; i < msg.points.size(); i++)
    {
        Point p;
        p.x = msg.points[i].x;
        p.y = msg.points[i].y;
        // p.l = msg.points[i].l;
        // p.r = msg.points[i].r;
        this->multi_path_road.push_back(p);
    }
}

void pure_pursuit::avoidance_callback(const std_msgs::Bool msg)
{
    this->ls_avoidance = msg.data;
}

void pure_pursuit::node_start(int argc, char *argv[])
{
    ros::init(argc, argv, "pure_pursuit");
    ros::NodeHandle n;

    int odom_ = -1; // lzw优化46_1117
    int slam_ = -1;
    int gps_ = -1;
    n.param<int>("ODOM_", odom_, -1);
    n.param<int>("SLAM_", slam_, -1);
    n.param<int>("GPS_", gps_, -1);
    ODOM = odom_;
    SLAM = slam_;
    GPS = gps_;
    // if ((SLAM != 1) || (GPS != 1)) // 实车
    if (SLAM != 1) // 实车
    {
        for (size_t i = 0; i < 20; i++)
        {
            ROS_WARN("Detected SLAM = %d, GPS = %d, ODOM = %d", SLAM, GPS, ODOM);
            ROS_WARN("SLAM != 1 or GPS != 1, please check the parameters in user.yaml!");
        }
        ros::shutdown();    // 关闭ROS节点，终止回调和定时器
        exit(EXIT_FAILURE); // 终止整个进程（需包含头文件 <cstdlib>）
    }
    pub_control_cmd = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10);                      // 发布cmd_vel
    marker_track_point_rviz = n.advertise<visualization_msgs::Marker>("track_point_rviz", 1); // 在rviz上显示追踪的点
    Ready_to_obstacle_pub = n.advertise<std_msgs::Bool>("/ready_to_obstacle", 1);             // 在rviz上显示追踪的点

    ros::Subscriber get_avoidance_control = n.subscribe("is_avoidance", 1, &pure_pursuit::avoidance_callback, this); // 接受避障
    ros::Subscriber get_path_control = n.subscribe("multi_path", 1, &pure_pursuit::multi_path_callback, this);       // 接受避障规划路径
    ros::Subscriber get_control = n.subscribe("to_control_all", 1, &pure_pursuit::get_control_callback, this);       // 接受local规划路径
    ros::Subscriber sub_gps = n.subscribe("/rear_post", 1, &pure_pursuit::odometryGetCallBack_GPS, this);            // 控制节点gps
    ros::Subscriber sub_slam = n.subscribe("/localization", 1, &pure_pursuit::odometryGetCallBack_slam, this);       //
    ros::Subscriber sub_odom = n.subscribe("/odom", 1, &pure_pursuit::odometryGetCallBack, this);                    //
    while (ros::ok)
    {
        ros::param::get("task", task);
        ros::param::get("stop_lidar", stop_lidar_mode);
        ros::param::get("stop_camera", stop_camera_mode);
        if (task == 1 || task == 9 || task == 11)
        {
            if (stop_lidar_mode == 0)
            {
                if (stop_camera_mode == 0)
                {
                    ros::Time time_now = ros::Time::now();
                    ros::spinOnce();
                    ros::Time time_now_2 = ros::Time::now();
                }
            }
        }
        geometry_msgs::Twist cmd_vel;
        if (task == -1)
        {
            cmd_vel.linear.y = 0;
            cmd_vel.linear.x = 0;
            cmd_vel.linear.z = 0;
            pub_control_cmd.publish(cmd_vel);
        }
        ros::param::set("first_flag", first_flag);
    }
}

int main(int argc, char *argv[])
{
    setlocale(LC_CTYPE, "zh_CN.utf8");
    pure_pursuit node;
    node.node_start(argc, argv);
    return 0;
}
