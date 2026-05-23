#include "ros/ros.h"
#include <geometry_msgs/PoseStamped.h>
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include <tf/tf.h>
using namespace std;
int get_end_point_flag=0;
int get_end_point_flag_shinei=0;
int task=0;
int flag3=0;
int main(int argc, char  *argv[])
{
    ros::init(argc, argv, "task_node");
    ros::NodeHandle nh;
    ros::Publisher end_pub=nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1);
    ros::param::set("first_flag",0);
        double raw, pitch, theta;
        tf::Quaternion q;

   

        geometry_msgs::PoseStamped msg;
        msg.header.frame_id="odom";
        msg.header.stamp= ros::Time::now();
        //一楼电梯口
        // msg.pose.position.x=8.2;
        // msg.pose.position.y=-8.45;
        // msg.pose.position.z = 0;


        // msg.pose.orientation.x = -0.02;
        // msg.pose.orientation.y = -0.012;
        // msg.pose.orientation.z = 0.997;
        // msg.pose.orientation.w = -0.0272;
        //三楼电梯口
        // msg.pose.position.x=-4.5;
        // msg.pose.position.y=0.16;
        // msg.pose.position.z = 0;


        // msg.pose.orientation.x = -0.02;
        // msg.pose.orientation.y = 0;
        // msg.pose.orientation.z = 0.999;
        // msg.pose.orientation.w = 0.031;
        // 玛多
        // msg.pose.position.x=12.61481;
        // msg.pose.position.y=-2.874;
        // msg.pose.position.z = 0;

        // msg.pose.orientation.x = 0.017;
        // msg.pose.orientation.y = -0.005;
        // msg.pose.orientation.z = -0.679;
        // msg.pose.orientation.w = 0.7340;//-3.00544
        bool first_flag=0;
        while(1){//first_flag
            cout<<"fabu__-"<<endl;
            ros::param::get("first_flag",first_flag);
            end_pub.publish(msg);
    
            string a="asdd";
            ros::param::set("q",a.c_str());
            // tf::quaternionMsgToTF(msg.pose.orientation, q);
            // tf::Matrix3x3(q).getRPY(raw, pitch, theta);
            // cout<<theta<<endl;
        }
     


    return 0;
}