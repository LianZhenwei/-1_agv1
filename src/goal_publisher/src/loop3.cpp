#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <string>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <visualization_msgs/Marker.h>
int GPS = -1;
int ODOM = -1;
int SLAM = -1;
struct State
{
    double x = 0;
    double y = 0;
    double yaw = 0;
    double speed = 0;
    double yawrate = 0;
};

class GoalPublisher
{
public:
    GoalPublisher() : goal_(0), last_goal_(0), goal1_loaded_(false), goal2_loaded_(false), goal3_loaded_(false),
                      goal0_loaded_(false), loop_count_(0), max_loops_(0), current_goal_(0), start_goal_(0), count_on_arrival_(false)
    {
        nh_ = ros::NodeHandle();
        int odom_ = -1;
        int slam_ = -1;
        int gps_ = -1;
        nh_.param<int>("ODOM_", odom_, -1);
        nh_.param<int>("SLAM_", slam_, -1);
        nh_.param<int>("GPS_", gps_, -1);
        ODOM = odom_;
        SLAM = slam_;
        GPS = gps_;
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
        pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 10, true);
        arrow_pub_ = nh_.advertise<visualization_msgs::Marker>("/goal_visualization_arrow", 10);
        cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
        sub_slam = nh_.subscribe("/localization", 1, &GoalPublisher::odometryGetCallBack_slam, this);
        sub_odom = nh_.subscribe("/odom", 1, &GoalPublisher::odometryGetCallBack, this);
        loadParameters();
    }
    void run()
    {
        ros::Rate rate(10);
        while (ros::ok())
        {
            int current_task = 0;
            nh_.getParam("task", current_task);
            nh_.getParam("goal", goal_);
            if (current_task == 1 && goal_ != last_goal_)
            {
                if (goal_ == 1 && goal1_loaded_)
                {
                    ros::param::set("to_goal", 1);
                    ros::Duration(1).sleep();
                    publishGoal(goal1_);
                    ROS_INFO("Publishing the FIRST goal (goal1)");
                    current_goal_ = 1;
                    start_goal_ = 1;
                }
                else if (goal_ == 2 && goal2_loaded_)
                {
                    ros::param::set("to_goal", 2);
                    ros::Duration(1).sleep();
                    publishGoal(goal2_);
                    ROS_INFO("Publishing the SECOND goal (goal2)");
                    current_goal_ = 2;
                    start_goal_ = 2;
                }
                else if (goal_ == 3 && goal3_loaded_)
                {
                    ros::param::set("to_goal", 3);
                    ros::Duration(1).sleep();
                    publishGoal(goal3_);
                    ROS_INFO("Publishing the THIRD goal (goal3)");
                    current_goal_ = 3;
                    start_goal_ = 3;
                }
                last_goal_ = goal_;
                loop_count_ = 0;
            }
            if (goal0_loaded_ && current_task == 1)
            {
                ros::param::set("to_goal", goal0_id_);
                ros::Duration(1).sleep();
                publishGoal(goal0_);
                if (max_loops_ == -1)
                {
                    ROS_INFO("Publishing next goal (Infinite loop, completed %d loops)", loop_count_ + 1);
                }
                else
                {
                    ROS_INFO("Publishing next goal (Loop %d/%d)", loop_count_ + 1, max_loops_);
                }
                current_goal_ = goal0_id_;
                goal0_loaded_ = false;
            }
            ros::spinOnce();
            rate.sleep();
        }
    }

private:
    void loadParameters()
    {
        nh_.param<int>("max_loops", max_loops_, 1);
        if (max_loops_ == -1)
        {
            ROS_INFO("Maximum loops set to: %d (Infinite loop mode)", max_loops_);
        }
        else
        {
            ROS_INFO("Maximum loops set to: %d", max_loops_);
        }
        nh_.param<double>("goal1_x", goal1_.pose.position.x, 12.61481);
        nh_.param<double>("goal1_y", goal1_.pose.position.y, -2.874);
        nh_.param<double>("goal1_z", goal1_.pose.position.z, 0.0);
        nh_.param<double>("goal1_qx", goal1_.pose.orientation.x, 0.017);
        nh_.param<double>("goal1_qy", goal1_.pose.orientation.y, -0.005);
        nh_.param<double>("goal1_qz", goal1_.pose.orientation.z, -0.680892);
        nh_.param<double>("goal1_qw", goal1_.pose.orientation.w, 0.732384);
        goal1_.header.frame_id = "map";
        goal1_loaded_ = true;
        nh_.param<double>("goal2_x", goal2_.pose.position.x, 0.0);
        nh_.param<double>("goal2_y", goal2_.pose.position.y, 0.0);
        nh_.param<double>("goal2_z", goal2_.pose.position.z, 0.0);
        nh_.param<double>("goal2_qx", goal2_.pose.orientation.x, 0.0);
        nh_.param<double>("goal2_qy", goal2_.pose.orientation.y, 0.0);
        nh_.param<double>("goal2_qz", goal2_.pose.orientation.z, 0.0);
        nh_.param<double>("goal2_qw", goal2_.pose.orientation.w, 1.0);
        goal2_.header.frame_id = "map";
        goal2_loaded_ = true;
        nh_.param<double>("goal3_x", goal3_.pose.position.x, 5.0);
        nh_.param<double>("goal3_y", goal3_.pose.position.y, 5.0);
        nh_.param<double>("goal3_z", goal3_.pose.position.z, 0.0);
        nh_.param<double>("goal3_qx", goal3_.pose.orientation.x, 0.0);
        nh_.param<double>("goal3_qy", goal3_.pose.orientation.y, 0.0);
        nh_.param<double>("goal3_qz", goal3_.pose.orientation.z, 0.0);
        nh_.param<double>("goal3_qw", goal3_.pose.orientation.w, 1.0);
        goal3_.header.frame_id = "map";
        goal3_loaded_ = true;
        ROS_INFO_STREAM("Loaded parameters for goal1: \n"
                        << "  position: (" << goal1_.pose.position.x << ", "
                        << goal1_.pose.position.y << ", " << goal1_.pose.position.z << ")\n"
                        << "  orientation: (" << goal1_.pose.orientation.x << ", "
                        << goal1_.pose.orientation.y << ", " << goal1_.pose.orientation.z << ", "
                        << goal1_.pose.orientation.w << ")");
        ROS_INFO_STREAM("Loaded parameters for goal2: \n"
                        << "  position: (" << goal2_.pose.position.x << ", "
                        << goal2_.pose.position.y << ", " << goal2_.pose.position.z << ")\n"
                        << "  orientation: (" << goal2_.pose.orientation.x << ", "
                        << goal2_.pose.orientation.y << ", " << goal2_.pose.orientation.z << ", "
                        << goal2_.pose.orientation.w << ")");
        ROS_INFO_STREAM("Loaded parameters for goal3: \n"
                        << "  position: (" << goal3_.pose.position.x << ", "
                        << goal3_.pose.position.y << ", " << goal3_.pose.position.z << ")\n"
                        << "  orientation: (" << goal3_.pose.orientation.x << ", "
                        << goal3_.pose.orientation.y << ", " << goal3_.pose.orientation.z << ", "
                        << goal3_.pose.orientation.w << ")");
    }

    void publishGoal(const geometry_msgs::PoseStamped &goal_template)
    {
        geometry_msgs::PoseStamped goal = goal_template;
        goal.header.stamp = ros::Time::now();
        pub_.publish(goal);
        ros::Duration(0.01).sleep();
        publishGoalArrow(goal);
    }

    void publishGoalArrow(const geometry_msgs::PoseStamped &goal)
    {
        visualization_msgs::Marker arrow;
        arrow.header = goal.header;
        arrow.ns = "goal_arrows";
        arrow.id = 0;
        arrow.type = visualization_msgs::Marker::ARROW;
        arrow.action = visualization_msgs::Marker::ADD;
        arrow.pose = goal.pose;
        arrow.scale.x = 2.8;
        arrow.scale.y = 0.1;
        arrow.scale.z = 0.2;
        arrow.color.r = 0.2f;
        arrow.color.g = 0.3f;
        arrow.color.b = 0.7f;
        arrow.color.a = 1.0;
        arrow.lifetime = ros::Duration();
        arrow_pub_.publish(arrow);
    }

    void publishZeroVelocity()
    {
        geometry_msgs::Twist cmd_vel;
        cmd_vel.linear.x = 0.0;
        cmd_vel.linear.y = 0.0;
        cmd_vel.linear.z = 0.0;
        cmd_vel.angular.x = 0.0;
        cmd_vel.angular.y = 0.0;
        cmd_vel.angular.z = 0.0;
        for (int i = 0; i < 5; i++)
        {
            cmd_vel_pub_.publish(cmd_vel);
            ros::Duration(0.1).sleep();
        }
        ROS_INFO("Published zero velocity command");
    }

    void odometryGetCallBack_slam(const nav_msgs::Odometry::ConstPtr odometry_msg)
    {
        if (SLAM)
        {
            double raw, pitch, theta;
            tf::Quaternion q;
            tf::quaternionMsgToTF(odometry_msg->pose.pose.orientation, q);
            tf::Matrix3x3(q).getRPY(raw, pitch, theta);
            this->vehicleState.yaw = theta;
            this->vehicleState.x = odometry_msg->pose.pose.position.x - cos(theta) * 1;
            this->vehicleState.y = odometry_msg->pose.pose.position.y - sin(theta) * 1;
            if (sqrt(pow(vehicleState.x - goal1_.pose.position.x, 2) +
                     pow(vehicleState.y - goal1_.pose.position.y, 2)) < 0.5)
            {
                if (!goal0_loaded_ && current_goal_ == 1)
                {
                    goal0_ = goal2_;
                    goal0_id_ = 2;
                    goal0_.header.frame_id = "map";
                    goal0_loaded_ = true;
                    ROS_INFO("Reached goal1, setting next goal to goal2");
                    if (goal0_id_ == start_goal_)
                        count_on_arrival_ = true;
                }
            }
            else if (sqrt(pow(vehicleState.x - goal2_.pose.position.x, 2) +
                          pow(vehicleState.y - goal2_.pose.position.y, 2)) < 0.5)
            {
                if (!goal0_loaded_ && current_goal_ == 2)
                {
                    goal0_ = goal3_;
                    goal0_id_ = 3;
                    goal0_.header.frame_id = "map";
                    goal0_loaded_ = true;
                    ROS_INFO("Reached goal2, setting next goal to goal3");
                    if (goal0_id_ == start_goal_)
                        count_on_arrival_ = true;
                }
            }
            else if (sqrt(pow(vehicleState.x - goal3_.pose.position.x, 2) +
                          pow(vehicleState.y - goal3_.pose.position.y, 2)) < 0.5)
            {
                if (!goal0_loaded_ && current_goal_ == 3)
                {
                    goal0_ = goal1_;
                    goal0_id_ = 1;
                    goal0_.header.frame_id = "map";
                    goal0_loaded_ = true;
                    ROS_INFO("Reached goal3, setting next goal to goal1");
                    if (goal0_id_ == start_goal_)
                        count_on_arrival_ = true;
                }
            }
            if (count_on_arrival_ && current_goal_ == start_goal_)
            {
                loop_count_++;
                count_on_arrival_ = false;
                if (max_loops_ == -1)
                {
                    ROS_INFO("Completed loop %d (Infinite loop mode)", loop_count_);
                }
                else
                {
                    ROS_INFO("Completed loop %d of %d", loop_count_, max_loops_);
                    if (loop_count_ >= max_loops_)
                    {
                        nh_.setParam("task", 11);
                        publishZeroVelocity();
                        ROS_INFO("Reached maximum loops. Stopping.");
                    }
                }
            }
        }
    }
    void odometryGetCallBack(const nav_msgs::Odometry::ConstPtr odometry_msg)
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
            if (sqrt(pow(vehicleState.x - goal1_.pose.position.x, 2) +
                     pow(vehicleState.y - goal1_.pose.position.y, 2)) < 0.5)
            {
                if (!goal0_loaded_ && current_goal_ == 1)
                {
                    goal0_ = goal2_;
                    goal0_id_ = 2;
                    goal0_.header.frame_id = "map";
                    goal0_loaded_ = true;
                    ROS_INFO("Reached goal1, setting next goal to goal2");
                    if (goal0_id_ == start_goal_)
                        count_on_arrival_ = true;
                }
            }
            else if (sqrt(pow(vehicleState.x - goal2_.pose.position.x, 2) +
                          pow(vehicleState.y - goal2_.pose.position.y, 2)) < 0.5)
            {
                if (!goal0_loaded_ && current_goal_ == 2)
                {
                    goal0_ = goal3_;
                    goal0_id_ = 3;
                    goal0_.header.frame_id = "map";
                    goal0_loaded_ = true;
                    ROS_INFO("Reached goal2, setting next goal to goal3");
                    if (goal0_id_ == start_goal_)
                        count_on_arrival_ = true;
                }
            }
            else if (sqrt(pow(vehicleState.x - goal3_.pose.position.x, 2) +
                          pow(vehicleState.y - goal3_.pose.position.y, 2)) < 0.5)
            {
                if (!goal0_loaded_ && current_goal_ == 3)
                {
                    goal0_ = goal1_;
                    goal0_id_ = 1;
                    goal0_.header.frame_id = "map";
                    goal0_loaded_ = true;
                    ROS_INFO("Reached goal3, setting next goal to goal1");
                    if (goal0_id_ == start_goal_)
                        count_on_arrival_ = true;
                }
            }
            if (count_on_arrival_ && current_goal_ == start_goal_)
            {
                loop_count_++;
                count_on_arrival_ = false;
                if (max_loops_ == -1)
                {
                    ROS_INFO("Completed loop %d (Infinite loop mode)", loop_count_);
                }
                else
                {
                    ROS_INFO("Completed loop %d of %d", loop_count_, max_loops_);
                    if (loop_count_ >= max_loops_)
                    {
                        nh_.setParam("task", 11);
                        ros::param::set("renwu_finish", 1);
                        ros::param::set("work_state", 1);
                        publishZeroVelocity();
                        ROS_INFO("Reached maximum loops. Stopping.");
                    }
                }
            }
        }
    }
    ros::NodeHandle nh_;
    ros::Publisher pub_;
    ros::Publisher arrow_pub_;
    ros::Publisher cmd_vel_pub_;
    ros::Subscriber sub_slam;
    ros::Subscriber sub_odom;
    int goal_;
    int last_goal_;
    State vehicleState;
    geometry_msgs::PoseStamped goal1_;
    geometry_msgs::PoseStamped goal2_;
    geometry_msgs::PoseStamped goal3_;
    geometry_msgs::PoseStamped goal0_;
    bool goal1_loaded_;
    bool goal2_loaded_;
    bool goal3_loaded_;
    bool goal0_loaded_;
    int loop_count_;
    int max_loops_;
    int current_goal_;
    int start_goal_;
    bool count_on_arrival_;
    int goal0_id_;
};
int main(int argc, char **argv)
{
    ros::init(argc, argv, "loop3");
    GoalPublisher publisher;
    publisher.run();
    return 0;
}
