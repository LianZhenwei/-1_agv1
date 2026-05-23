#include "include/headfile.h"
using namespace std;

float turn_limit=0.2;

struct Point
{
    double x;
    double y;
    double l;
    double r;
    double s;
    double theta;
};
struct State
{
    double x = 0;     // m
    double y = 0;     // m
    double yaw = 0;   // degree
    double speed = 0; // m/s
    double yawrate = 0;
};
struct Road
{
    int id;
    std::vector<int> pre;
    std::vector<int> beh;
    double distance;
    std::vector<Point> road_points;
};

//加载txt中路径
 std::vector<Road> load_direction_Road(const std::string fileName)
{ 
    std::vector<Road> roads;
    bool record_points = false;
    Road road;
    std::string line;
    std::string line3;
    std::ifstream fs;
    fs.open(fileName, std::ios::in);
    while (getline(fs, line)) {
        if (line.length() > 0) {
            std::stringstream ss(line);
            std::stringstream sss(line);
            std::string line2;
            int id;
            ss>>line2;
            if (line2 == "road")
            {   
                ss>>id;
                road.id = id;
                continue;
            }
        
            if (line2 == "point")
            {
                record_points = true;
                continue;
            }
            if (line2 != "pre" && record_points)
            {
             Point  point ;
             point.l=3;
             point.r=3;
             ss >> point.x >> point.y >> point.theta;
            //  point.x=-point.x;
             road.road_points.push_back(point); 
            }
    
            if (line2 == "pre")
            {
                int count = 0;
                while(getline(sss,line3,' ')){ 
                   count++;
                   if (count>=2)
                   {
                       std::stringstream ssss(line3);
                       int id;
                       ssss>>id;
                       road.pre.push_back(id);
                   }
                }
                record_points = false;
                continue;
            }
 
            if (line2 == "beh")
            {    
                int count = 0;
                while(getline(sss,line3,' ')){ 
                   count++;
                   if (count>=2)
                   {
                       std::stringstream ssss(line3);
                       int id;
                       ssss>>id;
                       road.beh.push_back(id);
                   }
                }
                
                record_points = false;
                roads.push_back(road); 
                road = {0};               
                continue;  
            }
        }   
    }
    
    fs.close();
    return roads;
}


class palletizing{
public:
    ros::Publisher pub_control_cmd_palletizing;
    State vehicleState;

    vector<Road> roads;
    vector<Point> track_road;
    int task=0;
    int stop_mode=0;
    int angle_pid_flag=0;

  void nodeStart(int argc, char **argv);
  void odometryGetCallBack_slam(const nav_msgs::Odometry::ConstPtr odometry_msg);
  void stop_deal();
};
void palletizing::stop_deal(){
    //进入停车控制模式
        float dis=sqrt(pow(vehicleState.x - track_road[track_road.size()-1].x, 2) + pow(vehicleState.y - track_road[track_road.size()-1].y, 2));
        float wheelAngle = atan2(track_road[track_road.size()-1].y- vehicleState.y , track_road[track_road.size()-1].x -vehicleState.x)-vehicleState.yaw; // 误差角度
        geometry_msgs::Twist cmd_vel;
        cout<<"angle_pid_flag"<<angle_pid_flag<<endl;
        if((abs(wheelAngle)*53.5>0.8 && dis<0.6) || angle_pid_flag){//走过终点后，进一次就必进
            angle_pid_flag=1;
            if(abs(track_road[track_road.size()-1].theta-vehicleState.yaw)<0.05 ){
             //普通寻迹逻辑
                cout<<"-------------------------------stop_over-----------------------"<<endl;
                for(int i=0;i<20;i++){
                    cmd_vel.linear.x = 0;
                    cmd_vel.linear.y = 0;
                    cmd_vel.angular.x=0;
                    pub_control_cmd_palletizing.publish(cmd_vel);
                    ros::param::set("renwu_finish",10);
                    ros::param::set("work_state",1);
                }
            }else{//角度调整
                cout<<"stop_222"<<endl;
                cmd_vel.linear.x = 0;
                cmd_vel.linear.y = 0;
                cmd_vel.angular.z=-PID_Realize1(&stop3_angle_PID, stop3_angle_pid, (track_road[track_road.size()-1].theta-vehicleState.yaw) * 100, 0);
                range_precote( cmd_vel.angular.z,0.1,-0.1);
                pub_control_cmd_palletizing.publish(cmd_vel);
            } 
        }else{//正常寻
            cout<<"stop_111"<<endl;
            geometry_msgs::Twist cmd_vel;
            cmd_vel.angular.z = 0;
            cmd_vel.linear.x = 0;
            cmd_vel.linear.y = 0.099998888;
            this->pub_control_cmd_palletizing.publish(cmd_vel);
        }
}
void palletizing::odometryGetCallBack_slam(const nav_msgs::Odometry::ConstPtr odometry_msg){
        ros::Rate loop_rate(10);
        double raw, pitch, theta;
        tf::Quaternion q;
        tf::quaternionMsgToTF(odometry_msg->pose.pose.orientation, q);
        tf::Matrix3x3(q).getRPY(raw, pitch, theta);
        this->vehicleState.yaw = theta;
        this->vehicleState.x =  odometry_msg->pose.pose.position.x - cos(theta) * (0.9) ;//
        this->vehicleState.y = odometry_msg->pose.pose.position.y - sin(theta) * (0.9) ;// 纵向

        if(this->vehicleState.yaw >3.14)  {this->vehicleState.yaw =this->vehicleState.yaw -6.28;}  
        if(this->vehicleState.yaw <-3.14)  {this->vehicleState.yaw =this->vehicleState.yaw +6.28;}  

        loop_rate.sleep();

        geometry_msgs::Twist cmd_vel;
        cmd_vel.linear.x = -0.1;
        cmd_vel.linear.y = 0;
        cmd_vel.angular.z=0;
        range_precote( cmd_vel.angular.z,0.1,-0.1);
        pub_control_cmd_palletizing.publish(cmd_vel);
        cout << "dis" << sqrt(pow(vehicleState.x - track_road[track_road.size() - 1].x, 2) + pow(vehicleState.y - track_road[track_road.size() - 1].y, 2)) << endl;
        if (sqrt(pow(vehicleState.x - track_road[track_road.size() - 1].x, 2) + pow(vehicleState.y - track_road[track_road.size() - 1].y, 2)) < 1)
        {
            stop_mode = 1;
        }
        if (stop_mode == 0)
        {
            geometry_msgs::Twist cmd_vel;
            cmd_vel.angular.z = 0;
            cmd_vel.linear.x = 0;
            cmd_vel.linear.y = 0.099998888;
            this->pub_control_cmd_palletizing.publish(cmd_vel);
        }
        else
        {
            stop_deal();
        }
}
void palletizing::nodeStart(int argc, char **argv){
    ros::init(argc, argv, "nodeStart_node");
    ros::NodeHandle nc;

    string roadMap_path = "/home/yt/agv/data/floor1.txt";//地图录制位置straight  slam_road123
    this->roads = load_direction_Road(roadMap_path);

    for(int i=0;i<8;i++){
        Point p;
        p.x=roads[0].road_points[i].x;
        p.y=roads[0].road_points[i].y;
        track_road.push_back(p);
    }
        Point p;
        p.x=12.662;
        p.y=-0.054;
        p.theta=1.57;
        track_road.push_back(p);

    this->pub_control_cmd_palletizing = nc.advertise<geometry_msgs::Twist>("/cmd_vel", 10);//发布cmd_vel
    ros::Subscriber sub_slam = nc.subscribe("/localization", 1, &palletizing::odometryGetCallBack_slam, this); // 

    while(ros::ok){
        ros::param::get("task",task);
        if(task == 10){
            ros::spinOnce();        
        }else{
            stop_mode=0;
            angle_pid_flag=0;
        }
         
    }
  }
int main(int argc, char  *argv[])
{
   palletizing node;
   node.nodeStart(argc,argv);
    return 0;
}
