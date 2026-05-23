#include "path_record.hpp"
#include <vector>
#include <ros/ros.h>
#include <tf/tf.h>
#include <fstream>
#include <ros/package.h>
#include <ros/node_handle.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64MultiArray.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <sys/time.h>
#include <time.h>
#include <cmath>
#include <iostream>
#include<sstream>
#include<string>
#include <unistd.h>
using namespace std;
int i = 2;

path_record::path_record(/* args */)
{
}

path_record::~path_record()
{
}

void path_record::record_path(const nav_msgs::Odometry::ConstPtr odometry_msg){
  char p;
  p='c';
   if (this->count == 0)
   {
    double raw, pitch, theta;
     tf::Quaternion q;
     tf::quaternionMsgToTF(odometry_msg->pose.pose.orientation, q);
     tf::Matrix3x3(q).getRPY(raw, pitch, theta);
      std::string roadMap_path = "/home/yt/agv/data/odom.txt";//地图录制位置
      FILE *fp_s;
      fp_s = fopen("/home/yt/agv/data/odom.txt","a");
      fprintf(fp_s, "%c %lf %lf %lf", p,odometry_msg->pose.pose.position.x, odometry_msg->pose.pose.position.y,odometry_msg->pose.pose.position.z);                   // Angle
      fprintf(fp_s, "\r\n");  
      fclose(fp_s);
      start_x =  odometry_msg->pose.pose.position.x;
      start_y =  odometry_msg->pose.pose.position.y;
      this->count++;
   }else{
    sample_distance = sample_distance+sqrt(pow((odometry_msg->pose.pose.position.x - start_x), 2) + pow((odometry_msg->pose.pose.position.y- start_y), 2) );
    start_x = odometry_msg->pose.pose.position.x;
    start_y = odometry_msg->pose.pose.position.y;
    cout<<odometry_msg->pose.pose.position.x<<endl;
    cout<<"odometry_msg->pose.pose.position.y"<<odometry_msg->pose.pose.position.y<<endl;
    if ( sample_distance >= S0)
    {
      double raw, pitch, theta;
     tf::Quaternion q;
     tf::quaternionMsgToTF(odometry_msg->pose.pose.orientation, q);
     tf::Matrix3x3(q).getRPY(raw, pitch, theta);
      FILE *fp_s;
      fp_s = fopen("/home/yt/agv/data/odom.txt","a");
      fprintf(fp_s, "%c %lf %lf %lf",p, odometry_msg->pose.pose.position.x, odometry_msg->pose.pose.position.y, odometry_msg->pose.pose.position.z);                   // Angle
      fprintf(fp_s, "\r\n");  
      fclose(fp_s);
      sample_distance = 0;
      this->count++;      
    } 
   }  
}

void path_record::record_path_GPS(const geometry_msgs::PoseStamped::ConstPtr odometry_msg){
  cout<<"qqqqqqqqqqqqqq"<<endl;
  char p;
  p='c';
   if (this->count == 0)
   {
    double raw, pitch, theta;
     tf::Quaternion q;
     tf::quaternionMsgToTF(odometry_msg->pose.orientation, q);
     tf::Matrix3x3(q).getRPY(raw, pitch, theta);
      std::string roadMap_path = "/home/yt/agv/data/GPS.txt";//地图录制位置
      FILE *fp_s;
      fp_s = fopen("/home/yt/agv/data/GPS.txt","a");
      fprintf(fp_s, "%c %lf %lf %lf", p,-odometry_msg->pose.position.x, odometry_msg->pose.position.y,theta);                   // Angle
      fprintf(fp_s, "\r\n");  
      fclose(fp_s);
      start_x =  odometry_msg->pose.position.x;
      start_y =  odometry_msg->pose.position.y;
      this->count++;
   }else{
    sample_distance = sample_distance+sqrt(pow((-odometry_msg->pose.position.x - start_x), 2) + pow((odometry_msg->pose.position.y- start_y), 2) );
    start_x = odometry_msg->pose.position.x;
    start_y = odometry_msg->pose.position.y;
       ROS_INFO_STREAM (" /////////////////////// sample_distance   == "<< sample_distance   ); 
       ROS_INFO_STREAM (" ///////////////////////  start_x    == "<<  start_x  ); 
        ROS_INFO_STREAM (" ///////////////////////  odometry_msg->pose.pose.position.x     == "<<  odometry_msg->pose.position.x   ); 
    if ( sample_distance >= S0)
    {
     double raw, pitch, theta;
     tf::Quaternion q;
     tf::quaternionMsgToTF(odometry_msg->pose.orientation, q);
     tf::Matrix3x3(q).getRPY(raw, pitch, theta);
     FILE *fp_s;
     fp_s = fopen("/home/yt/agv/data/GPS.txt","a");
     fprintf(fp_s, "%c %lf %lf %lf",p, odometry_msg->pose.position.x, odometry_msg->pose.position.y,theta);                   // Angle
     fprintf(fp_s, "\r\n");  
     fclose(fp_s);
      sample_distance = 0;
      this->count++;      
    } 
   }  
}

void path_record::record_path2(const nav_msgs::Odometry::ConstPtr odometry_msg){
     cout<<"/////start_path_record////////"<<endl;
   //   ros::Rate loop_rate(10);
     char p;
     p='c';
     double raw, pitch, theta;
     tf::Quaternion q;
     tf::quaternionMsgToTF(odometry_msg->pose.pose.orientation, q);
     tf::Matrix3x3(q).getRPY(raw, pitch, theta);
      FILE *fp_s;
      fp_s = fopen("/home/lwh/Lpp_files/data/odom3_road_path.txt","a");
      fprintf(fp_s, "%c %lf %lf %lf", p, odometry_msg->pose.pose.position.x, odometry_msg->pose.pose.position.y,theta);                   // Angle
      fprintf(fp_s, "\r\n");  
      fclose(fp_s);
     // loop_rate.sleep();
}

void path_record::odometryGetCallBack_slam(const nav_msgs::Odometry::ConstPtr odometry_msg){
  char p;
  p='c';

   if (this->count == 0)
   {
    double raw, pitch, theta;
     tf::Quaternion q;
     tf::quaternionMsgToTF(odometry_msg->pose.pose.orientation, q);
     tf::Matrix3x3(q).getRPY(raw, pitch, theta);
      std::string roadMap_path = "/home/agv/agv/data/slam.txt";//地图录制位置
      FILE *fp_s;
      fp_s = fopen("/home/agv/agv/data/slam.txt","a");
      fprintf(fp_s, "%c %lf %lf %lf", p,odometry_msg->pose.pose.position.x,-odometry_msg->pose.pose.position.y,theta);                   // Angle
      fprintf(fp_s, "\r\n");  
      fclose(fp_s);
      start_x = odometry_msg->pose.pose.position.x;
      start_y=-odometry_msg->pose.pose.position.y;
      this->count++;
   }else{
   float  dis_by=sqrt(pow(( odometry_msg->pose.pose.position.x - start_x), 2) + pow((odometry_msg->pose.pose.position.y- start_y), 2) );
    if(abs(dis_by) > 0.5) {dis_by=0;}
    sample_distance = sample_distance+dis_by;

   // sample_distance = sample_distance+sqrt(pow((odometry_msg->pose.pose.position.x - start_x), 2) + pow((-odometry_msg->pose.pose.position.y- start_y), 2) );
    cout<<"start_x"<<start_x<<endl;
    cout<<"start_y"<<start_y<<endl;
    cout<<"odometry_msg->pose.pose.position.x"<<odometry_msg->pose.pose.position.x<<endl;
    cout<<"odometry_msg->pose.pose.position.y"<<odometry_msg->pose.pose.position.y<<endl;
    start_x =odometry_msg->pose.pose.position.x;
    start_y =-odometry_msg->pose.pose.position.y;

    if ( sample_distance >= S0)
    {
      cout<<"qqqqqqqqqqqqqq"<<endl;
      double raw, pitch, theta;
     tf::Quaternion q;
     tf::quaternionMsgToTF(odometry_msg->pose.pose.orientation, q);
     tf::Matrix3x3(q).getRPY(raw, pitch, theta);
      FILE *fp_s;
      fp_s = fopen("/home/agv/agv/data/slam.txt","a");
      fprintf(fp_s, "%c %lf %lf %lf",p, odometry_msg->pose.pose.position.x, -odometry_msg->pose.pose.position.y,theta);                   // Angle
      fprintf(fp_s, "\r\n");  
      fclose(fp_s);
      sample_distance = 0;
      this->count++;      
    } 
   }  
}

void path_record::odometryGetCallBack_slam2(const nav_msgs::Odometry::ConstPtr odometry_msg){
  char p;
  p='c';
  ros::Rate rate(0.5);
  rate.sleep();
   if (this->count == 0)
   {  
    double raw, pitch, theta;
     tf::Quaternion q;
     tf::quaternionMsgToTF( odometry_msg->pose.pose.orientation, q);
     tf::Matrix3x3(q).getRPY(raw, pitch, theta);
      std::string roadMap_path = "/home/agv/agv/data/slam1.txt";//地图录制位置
      FILE *fp_s;
      fp_s = fopen("/home/agv/agv/data/slam1.txt","a");
      fprintf(fp_s, "%c %lf %lf %lf", p,   odometry_msg->pose.pose.position.x,   odometry_msg->pose.pose.position.y,theta);                   // Angle
      fprintf(fp_s, "\r\n");  
      fclose(fp_s);
      start_x =     odometry_msg->pose.pose.position.x;
      start_y=    odometry_msg->pose.pose.position.y;
      this->count++;
   }else{
   
    float  dis_by=sqrt(pow((   odometry_msg->pose.pose.position.x - start_x), 2) + pow(( odometry_msg->pose.pose.position.y- start_y), 2) );
    if(abs(dis_by) < 0.1) {dis_by=0;}
    sample_distance = sample_distance+dis_by;

    cout<<"start_x"<<start_x<<endl;
    cout<<"start_y"<<start_y<<endl;
    cout<<"odometry_msg->pose.pose.position.x"<<   odometry_msg->pose.pose.position.x<<endl;
    cout<<"odometry_msg->pose.pose.position.y"<<   odometry_msg->pose.pose.position.y<<endl;
    cout<<"www"<<sample_distance<<endl;
    start_x = odometry_msg->pose.pose.position.x;
    start_y =  odometry_msg->pose.pose.position.y;

    if ( sample_distance >= S0)
    {
      cout<<"qqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqq"<<endl;
      double raw, pitch, theta;
     tf::Quaternion q;
     tf::quaternionMsgToTF(  odometry_msg->pose.pose.orientation, q);
     tf::Matrix3x3(q).getRPY(raw, pitch, theta);
      FILE *fp_s;
      fp_s = fopen("/home/agv/agv/data/slam1.txt","a");
      fprintf(fp_s, "%c %lf %lf %lf",p, odometry_msg->pose.pose.position.x,  odometry_msg->pose.pose.position.y,theta);                   // Angle
      fprintf(fp_s, "\r\n");  
      fclose(fp_s);
      sample_distance = 0;
      this->count++;      
    } 
   }  
}
void path_record::odometryGetCallBack_slam3(const nav_msgs::Odometry::ConstPtr odometry_msg){
 ros::Rate rate(5);
 rate.sleep();
  char p;
  p='c';
  if (this->count == 0)
  {
    double raw, pitch, theta;
     tf::Quaternion q;
     tf::quaternionMsgToTF(odometry_msg->pose.pose.orientation, q);
     tf::Matrix3x3(q).getRPY(raw, pitch, theta);
      std::string roadMap_path = "/home/yt/agv/txt/000path_record.txt";//地图录制位置
      //std::string roadMap_path_2 = "/home/yt/agv/data/shiyan3_10.txt";
      FILE *fp_s;
      FILE *fp_s_2;
      fp_s = fopen("/home/yt/agv/txt/000path_record.txt","a");
      fprintf(fp_s, "%c %lf %lf %lf", p,odometry_msg->pose.pose.position.x,odometry_msg->pose.pose.position.y,theta);                   // Angle
      fprintf(fp_s, "\r\n");  
      fclose(fp_s);
      // fp_s_2 = fopen("/home/yt/agv/data/shiyan3_10.txt","a");
      // fprintf(fp_s_2, " %lf %lf %lf", odometry_msg->pose.pose.position.x,odometry_msg->pose.pose.position.y,theta);                   // Angle
      // fprintf(fp_s_2, "\r\n");  
      // fclose(fp_s_2);


      start_x = odometry_msg->pose.pose.position.x;
      start_y = odometry_msg->pose.pose.position.y;
      this->count++;
   }else{
    float dis_by=sqrt(pow((odometry_msg->pose.pose.position.x - start_x), 2) + pow((odometry_msg->pose.pose.position.y- start_y), 2) );
    if(dis_by<0.007){
      dis_by=0;
    }
    sample_distance = sample_distance+dis_by;
    
    start_x =odometry_msg->pose.pose.position.x;
    start_y =odometry_msg->pose.pose.position.y;

    if ( sample_distance >= S0)
    {
        cout<<"qqqqqqqqqqqqqq"<<endl;
      double raw, pitch, theta;
     tf::Quaternion q;
     tf::quaternionMsgToTF(odometry_msg->pose.pose.orientation, q);
     tf::Matrix3x3(q).getRPY(raw, pitch, theta);
      FILE *fp_s;
      FILE *fp_s_2;
      fp_s = fopen("/home/yt/agv/txt/000path_record.txt","a");
      fprintf(fp_s, "%c %lf %lf %lf",p, odometry_msg->pose.pose.position.x,odometry_msg->pose.pose.position.y,theta);                   // Angle
      fprintf(fp_s, "\r\n");  
      fclose(fp_s);
      // fp_s_2 = fopen("/home/yt/agv/data/shiyan3_10.txt","a");
      // fprintf(fp_s_2, "%d %lf %lf %lf", i ,odometry_msg->pose.pose.position.x,odometry_msg->pose.pose.position.y,theta);                   // Angle
      // fprintf(fp_s_2, "\r\n");  
      // fclose(fp_s_2);
      i = i + 1;
      sample_distance = 0;
      this->count++;      

    } 
   }  
/////////////////////////////////////////////
}

void path_record::nodeStart(int argc, char **argv){
    ros::init(argc, argv, "path_record");
    ros::NodeHandle nc;
     count = 0;
    // 订阅相关节点
    
    //  ros::Subscriber sub_odom = nc.subscribe("/odom", 1, &path_record::record_path, this);//录制轨迹话题小车里程计
    // ros::Subscriber sub_odom = nc.subscribe("/rear_post", 1, &path_record::record_path_GPS, this);//录制轨迹话题GPS
      ros::Subscriber sub_odom = nc.subscribe("/localization", 1, &path_record::odometryGetCallBack_slam3, this); // 控制节点gps
      //ros::Subscriber sub_odom_2 = nc.subscribe("/localization", 1, &path_record::odometryGetCallBack_slam3_2, this); // 控制节点gps
      ros::spin();
}

int main(int argc, char *argv[])
{
    
    path_record node;
    node.nodeStart(argc, argv);
    return(0);
}

// roscore
// 开启定位节点【作用是定位节点会实时发布“当前车在地图坐标系下的坐标(x, y, z)和车自身的朝向角”到话题"/localization"】
// rosrun path_record path_record
// 再将遥控器设为手动（即遥控器控制车），再使用遥控器控制车运动，车每运动0.1米path_record就会将车的x、y、theta记录到文件/home/yt/agv/txt/000path_record.txt里一次
// 当Ctrl + C结束path_record节点运行后，车的运动轨迹就记录完毕了