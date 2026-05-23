#include "ros/ros.h"
#include <std_msgs/Bool.h>
#include <std_msgs/Float64MultiArray.h>
#include <iostream>
#include <nav_msgs/Odometry.h>
using namespace std;
struct obs_fromcamera{
    double pos_y;
    double pos_x;
    double pos_y_last = 0;
    double pos_x_last = 0;
    double size_x;
    double size_y;
    double vel_x;
    double vel_y;
    int id;
};
class camera_obs{
public:
    ros::Publisher pub_control_cmd;
    vector<obs_fromcamera> obs_vec;
    void node_start(int argc, char *argv[]);
    void cameraobsGetCallBack(const std_msgs::Float64MultiArray& msg);
};
void camera_obs::cameraobsGetCallBack(const std_msgs::Float64MultiArray& msg)//这个貌似是接受传递过来的障碍物的信息
{
    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = 0;
    cmd_vel.linear.y = 0;
    cmd_vel.angular.z = 0;

    int num=msg.data.size()/3;
    int num_i=0;
    for(int i=0;i<num;i++)
    {
        num_i++;
        obs_fromcamera obs;//定义一个障碍物的机构体
        obs.pos_x = msg.data[i * 3 + 2];//第三个数是远离摄像头的方向
        obs.pos_y = msg.data[i * 3 + 0];
        cout<<"julirendezhixian"<<obs.pos_x<<endl;
        cout<<"julirendehengxiang"<<obs.pos_y<<endl;
        if(abs(obs.pos_x)<1.5 && abs(obs.pos_y)<0.8)
        {
            ROS_WARN("停车! ");
            ros::param::set("stop_camera", 1);
            // 向 cmd_vel 发送速度控制
            pub_control_cmd.publish(cmd_vel);
            break;
        }
    }
    if(num_i==num)
    {
        int stop_number_camera = 0;
        ros::param::get("stop_camera_number", stop_number_camera);
        stop_number_camera = stop_number_camera + 1;
        ros::param::set("stop_camera_number", stop_number_camera);
        if(stop_number_camera >= 15)
        {
            ros::param::set("stop_camera", 0); // 发送过于随意
            ros::param::set("stop_camera_number", 0);
            cout<<"huifu---------------------------------"<<endl;
        }
    }

}

void camera_obs::node_start(int argc, char  *argv[]){
    ros::init(argc, argv, "camera_obs");
    ros::NodeHandle n;
    pub_control_cmd = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10);         // 发布cmd_vel
    ros::Subscriber sub_camera_obs=n.subscribe("/front_camera/detect_result_position", 1, &camera_obs::cameraobsGetCallBack, this); //进入相机检测到结果的回调
    while(ros::ok)
    {
        ros::spinOnce(); 
    }

}
int main(int argc, char  *argv[])
{
    camera_obs node;
    node.node_start(argc,argv);
    return 0;
}