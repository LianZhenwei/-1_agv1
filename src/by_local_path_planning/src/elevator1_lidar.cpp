#include "include/headfile.h"
#include <std_msgs/Float64MultiArray.h>
struct erweima{
  double pos_x;
  double pos_y;
  double pos_z;
  double theta;
};
int second_erweima=0;
int task8_count=0;
int task=0;
double angle_init=0;
bool first_flag=0;
float acc_z = 0;
float angle = 0;


bool over1=0;
int over1_count=0;
int stop_count=0;
int stop_flag=0;
int guandaoover_flag=0;
int guandao_count=0;
float dis_globle=100;
class erweima_class {
public:
    bool if_flag = 1;
    bool turn_flag_erweima = 0;
    bool turn_flag_erweima_x=0;
    bool over_flag=0;
    int task_flag=0;

    ros::Publisher pub_control_cmd_erweima;
    ros::Publisher qr_code_erweima;
    void nodeStart(int argc, char  *argv[]);
    void CallBack_lidar(const std_msgs::Float64MultiArray::ConstPtr& msg);
};

// const std_msgs::Float64MultiArray::ConstPtr& msg
void erweima_class::CallBack_lidar(const std_msgs::Float64MultiArray::ConstPtr& msg){
  ros::Rate rate(10);
  rate.sleep();
  geometry_msgs::Twist cmd_vel;
 
  erweima my_erweima;

  float control_erweima_y, dis;
  if(msg->data.size() == 4)
  {
  
    control_erweima_y = msg->data[1];//若左偏就加0.06
    my_erweima.theta = msg->data[2] ;
    dis = msg->data[0];
    cout<<"---------------dis:"<<dis<<endl;
    cout<<"control_erweima_y-------------------:"<<control_erweima_y<<endl;
    cout<<"my_erweima.theta---------------------:"<< my_erweima.theta <<endl;
  

  if(!over1 && task==6 )
  {
    if (if_flag)
    {
      cout<<"------------------jinrudao if: "<<dis<<endl;
      cout<<"---------------------if_flag: "<<if_flag<<endl;
      if((turn_flag_erweima == 0 && turn_flag_erweima_x == 0) ||  ((my_erweima.theta) > 0.2 || (my_erweima.theta) < -0.2)){
        if ((my_erweima.theta) > 0.04 || (my_erweima.theta) < -0.04)
        {
          turn_flag_erweima = 1;
        }
        if ((control_erweima_y) > 0.1 || (control_erweima_y) < -0.1)
        {
          turn_flag_erweima_x = 1;
        }
      }
      /////////////////////////////////////////
      if ((control_erweima_y) < 0.05 && (control_erweima_y) > -0.05)
      {
        turn_flag_erweima_x = 0;
      }
      if ((my_erweima.theta) < 0.02 && (my_erweima.theta) > -0.02)
      {
        turn_flag_erweima = 0;
      }
      if (!turn_flag_erweima && !turn_flag_erweima_x)
      {
        over_flag = 1;
      }else{
         over_flag = 0;
         over1_count=0;
      }
    } 
    if(turn_flag_erweima_x)
    {
      if(control_erweima_y>0){
        cmd_vel.linear.y=-0.05;
      }else{
        cmd_vel.linear.y=0.05;
      }
        cmd_vel.linear.x = 0;
        cmd_vel.angular.z = 0;
    }

    if(turn_flag_erweima)
    {
        cmd_vel.linear.x = 0;
        cmd_vel.linear.y = 0;
        cmd_vel.angular.z=PID_Realize1(&Direct_angle_PID, Direct_angle, (my_erweima.theta) * 100, 0);
        range_precote( cmd_vel.angular.z,0.1,-0.1);
    }

    if(over_flag){
      cmd_vel.linear.x = 0;
      cmd_vel.linear.y = 0.0;
      cmd_vel.angular.z = 0;
      pub_control_cmd_erweima.publish(cmd_vel);
      over1_count++;
      if(over1_count>20){
        over1 = 1;
        ros::param::set("renwu_finish",6);
        ros::param::set("work_state",1);
        ros::param::set("lidar_elevator_result",1);
      }
    }
    if(!over_flag){//!over_flag
      pub_control_cmd_erweima.publish(cmd_vel);
    }
  }
  }else{
    return;
  }

}
void erweima_class::nodeStart(int argc, char  *argv[]){
    ros::init(argc, argv, "jindianti");
    ros::NodeHandle nc;

    this->pub_control_cmd_erweima = nc.advertise<geometry_msgs::Twist>("/cmd_vel", 10);//发布cmd_vel
    ros::Subscriber sub_lidar=nc.subscribe("/elevator_result", 1, &erweima_class::CallBack_lidar, this);

 
    while (ros::ok)
    {
      ros::param::get("task",task);
      if(task == 6 || task == 7 || task == 8)
      {

        ros::spinOnce();
        geometry_msgs::Twist cmd_vel;
        if(task == 8)
        {
          if(task8_count <130)
          {
            ros::Rate rate(10);
            rate.sleep();
            task8_count++;
            cmd_vel.linear.y=0;
            cmd_vel.linear.x=0.2;
            cmd_vel.angular.z=0;
            pub_control_cmd_erweima.publish(cmd_vel);
          }
          else
          {
            cmd_vel.linear.y=0;
            cmd_vel.linear.x=0;
            cmd_vel.angular.z=0;
            pub_control_cmd_erweima.publish(cmd_vel);
     
            if(task_flag == 0){
              ros::param::set("renwu_finish",8);
              ros::param::set("work_state",1);
              task_flag=1;
            }   
          }
        }
      }
      else
      {
        task8_count=0;
        task_flag=0;
      }
    }
}

int main(int argc, char  *argv[])
{
    erweima_class node;
    PID_Parameter_Init(&Direct_x_PID);
    PID_Parameter_Init(&Direct_angle_PID);
    PID_Parameter_Init(&straight_PID);
    node.nodeStart(argc, argv);
    return 0;
}

