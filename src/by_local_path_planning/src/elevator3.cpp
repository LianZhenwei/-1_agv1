#include "include/headfile.h"
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
    int have_erweima_flag=0; 
    bool if_flag = 0;
    bool turn_flag_erweima = 0;
    bool turn_flag_erweima_x=0;
    bool over_flag=0;
    int task_flag=0;
    ros::Publisher pub_control_cmd_erweima;
    ros::Publisher qr_code_erweima;
    void nodeStart(int argc, char  *argv[]);
    void erweimaGetCallBack_erweima(const apriltag_ros::AprilTagDetectionArray& msg);
    void timer_cb1 ( const ros::TimerEvent & );
};
void erweima_class::erweimaGetCallBack_erweima(const apriltag_ros::AprilTagDetectionArray& msg){

  ros::Rate rate(10);
  rate.sleep();
  geometry_msgs::Twist cmd_vel;
  erweima my_erweima,my_erweima2;
  std::vector<Box_2d> erweima_;
  double theta, theta2,raw, pitch,raw2,pitch2;
  float control_erweima_y, dis, control_erweima_y2,dis2;
  if(msg.detections.size()>0){

    for(int i=0;i<msg.detections.size(); ++i){
       if(msg.detections[i].id[0]==12){
          tf::Quaternion q;
          tf::quaternionMsgToTF(msg.detections[i].pose.pose.pose.orientation, q);
          tf::Matrix3x3(q).getRPY(raw, pitch, theta);
          control_erweima_y2 = msg.detections[i].pose.pose.pose.position.x-0.16;//若左偏就加0.06
          my_erweima2.theta = pitch;
          dis2 = sqrt(pow(msg.detections[i].pose.pose.pose.position.x, 2) + pow(msg.detections[i].pose.pose.pose.position.z, 2)) ;
          cout<<"control_erweima_y2:"<<control_erweima_y2<<endl;
          cout<<"my_erweima2.theta:"<<my_erweima2.theta<<endl;
          cout<<"dis2:"<<dis2<<endl;
          have_erweima_flag=1;
       }
    }
     
    
    erweima_.clear();
    Box_2d obs1({(float) my_erweima.pos_x,(float)  my_erweima.pos_y}, 0.0,0.8,0.8, 0);
    erweima_.emplace_back(obs1);
    erweima_rviz(qr_code_erweima,erweima_);

  if(0){//!over1 && task == 6
    if ((dis2 < 3) || if_flag)
    {
      if((turn_flag_erweima == 0 && turn_flag_erweima_x == 0) ||  ((my_erweima2.theta) > 0.2 || (my_erweima2.theta) < -0.2)){
        if ((my_erweima2.theta) > 0.04 || (my_erweima2.theta) < -0.04)
        {
          turn_flag_erweima = 1;
        }
        if ((control_erweima_y2) > 0.1 || (control_erweima_y2) < -0.1)
        {
          turn_flag_erweima_x = 1;
        }
      }
      /////////////////////////////////////////
      if ((control_erweima_y2) < 0.04 && (control_erweima_y2) > -0.04)
      {
        turn_flag_erweima_x = 0;
      }
      if ((my_erweima2.theta) < 0.02 && (my_erweima2.theta) > -0.02)
      {
        turn_flag_erweima = 0;
      }
      if (!turn_flag_erweima && !turn_flag_erweima_x && dis<6)
      {
        over_flag = 1;
      }else{
         over_flag = 0;
         over1_count=0;
      }
      if_flag = 1; // 保证只要进一次后面就必进了
    }else{//yuan ju li pid
        cmd_vel.angular.z=PID_Realize1(&Direct_angle_PID, Direct_angle, (control_erweima_y2 )* 100, 0);
        cmd_vel.linear.y = 0;
        cmd_vel.linear.x = 0.2;
        pub_control_cmd_erweima.publish(cmd_vel);

      }  
            if(turn_flag_erweima_x){
              if(control_erweima_y2>0){
                cmd_vel.linear.y=-0.05;
              }else{
                cmd_vel.linear.y=0.05;
              }
                cmd_vel.linear.x = 0;
                cmd_vel.angular.z = 0;
            }

            if(turn_flag_erweima){
                cmd_vel.linear.x = 0;
                cmd_vel.linear.y = 0;
                cmd_vel.angular.z=PID_Realize1(&Direct_angle_PID, Direct_angle, (my_erweima2.theta) * 100, 0);
                range_precote( cmd_vel.angular.z,0.1,-0.1);
            }

            if(over_flag){
              cmd_vel.linear.x = 0;
              cmd_vel.linear.y = 0.0;
              cmd_vel.angular.x = 0;
              pub_control_cmd_erweima.publish(cmd_vel);
              over1_count++;
              if(over1_count>12){
                over1 = 1;
                ros::param::set("renwu_finish",6);
                ros::param::set("work_state",1);
              }
            }
            if(!over_flag){//!over_flag
              pub_control_cmd_erweima.publish(cmd_vel);
            }
    }else if(task == 7 && have_erweima_flag){//第二个二维码
          cmd_vel.angular.z = PID_Realize1(&straight_PID_second, straight_second, (control_erweima_y2) * 100, 0);
          range_precote(cmd_vel.angular.z,0.3,-0.3);
          cmd_vel.linear.x = 0.2;
          cmd_vel.linear.y = 0;
          if(dis2<=0.58 && have_erweima_flag){
                cmd_vel.linear.y=0;
                cmd_vel.linear.x=0;
                cmd_vel.angular.x=0;
                pub_control_cmd_erweima.publish(cmd_vel);
                ros::param::set("renwu_finish",7);
                ros::param::set("work_state",1);
            }
          pub_control_cmd_erweima.publish(cmd_vel);
    }else if(task == 6){
        ros::param::set("renwu_finish",6);
        ros::param::set("work_state",1);
    }
        // cout << "dis" << dis << endl;
        // cout << "x " << control_erweima_y << endl;
        // // cout << "theat " << my_erweima.theta << endl;
        // cout << "turn_flag_erweima " << turn_flag_erweima << endl;
        // cout << "turn_flag_erweima_x " << turn_flag_erweima_x << endl;

        // cout << "over_flag" << over_flag << endl;
        // cout<<"over1_count "<<over1_count<<endl;
  
        // cout<<"dis2 "<<dis2<<endl;
        // cout << "control_erweima_y2 " << control_erweima_y2 << endl;
        // cout << " my_erweima2.theta " <<  my_erweima2.theta << endl;

  }

}
  void erweima_class::nodeStart(int argc, char  *argv[]){
    ros::init(argc, argv, "jindianti3");
    ros::NodeHandle nc;

    this->pub_control_cmd_erweima = nc.advertise<geometry_msgs::Twist>("/cmd_vel", 10);//发布cmd_vel
    this->qr_code_erweima = nc.advertise<visualization_msgs::Marker>("erweima", 1);// 二维码
 
    ros::Subscriber sub_camera_erweima=nc.subscribe("/tag_detections", 1, &erweima_class::erweimaGetCallBack_erweima, this);

    while (ros::ok)
    {
        ros::param::get("task",task);
      if(task == 6 || task==7 || task == 8)
      {
        ros::spinOnce();
        geometry_msgs::Twist cmd_vel;
        if(task == 8)
        {
          if(task8_count <90){
            ros::Rate rate(10);
            rate.sleep();
            task8_count++;
            cout<<"task8:::::::::::::"<<task8_count<<endl;
            cmd_vel.linear.y=0;
            cmd_vel.linear.x=0.2;
            cmd_vel.angular.z=0;
            pub_control_cmd_erweima.publish(cmd_vel);
          }
          else{
            cmd_vel.linear.y=0;
            cmd_vel.linear.x=0;
            cmd_vel.angular.z=0;
            pub_control_cmd_erweima.publish(cmd_vel);
            if(task_flag== 0){
              task_flag=1;
              ros::param::set("renwu_finish",8);
              ros::param::set("work_state",1);
            }      
          }
        }
      }else{
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

