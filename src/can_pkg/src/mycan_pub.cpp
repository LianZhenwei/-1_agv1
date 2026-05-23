#include "ros/ros.h"
#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <pthread.h>
#include "include/controlcan.h"

#include <ctime>
#include <cstdlib>
#include "unistd.h"
#include <geometry_msgs/TwistStamped.h>
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "deal.h"

#include <nav_msgs/Odometry.h>
#include <stdlib.h>

#include "lvbo.h"

// ros::Publisher odom_pub ;
void range_precote(auto &val, double max, double min)
{
    if (val > max)
    {
        val = max;
    }
    if (val < min)
    {
        val = min;
    }
}


float dianchi=0;
float kongzhiquanxian=0;
int fuwei = 0;
int fuwei_last = 0;
int charge = 1;
int chubianpingbi = 0;
int dadeng = 0;

// 填充Odometry消息
nav_msgs::Odometry odom;
ros::Time current_time;
VCI_BOARD_INFO pInfo;//用来获取设备信息。
int count_can=0;//数据列表中，用来存储列表序号。
VCI_BOARD_INFO pInfo1 [50];

std::string Number2HexStr( uint32_t mData )
{
	std::stringstream ss;

	ss << std::hex << std::setw(2) << std::setfill('0') << (int)mData;

	std::string hex_str = ss.str();

	return hex_str;
}
std::string HexToString(const char* hex, size_t size) {  

    std::stringstream ss;  

    for (size_t i = 0; i < size; ++i) {  

        ss << std::hex << std::setw(2) << std::setfill('0') << (int)hex[i];  

    }  

    return ss.str();  

}

void timer_cb1 ( const ros::TimerEvent & )
{
	ros::param::get("charge",charge);
	ros::param::get("chubianpingbi",chubianpingbi);
	ros::param::get("dadeng",dadeng);
	VCI_CAN_OBJ send[1];
	//需要发送的帧，结构体设置
	send[0].ID=0x029B;
	send[0].SendType=0;
	send[0].RemoteFlag=0;
	send[0].ExternFlag=0;
	send[0].DataLen=8;

	send[0].Data[0] = 33;//rf 90
	send[0].Data[1] = 80;//lf
	send[0].Data[2] = 1;//br
	send[0].Data[3] = charge;//bl
	//data3是0，充电；data3是1，就是收回充电

	//send[0].Data[4] = chubianpingbi; // 触边屏蔽，1是屏蔽，0是不屏蔽
	send[0].Data[4] = 0;
	send[0].Data[5] = dadeng; // 大灯，0是开，1是关闭
	send[0].Data[6] = 0;
	send[0].Data[7] = 0;

	if (VCI_Transmit(VCI_USBCAN2, 0, 0, send, 1) == 1){}
}
void timer_cb2 ( const ros::TimerEvent & )
{
}


void *receive_func(void* param)  //接收线程。
{
	int reclen=0;
	VCI_CAN_OBJ rec[3000];//接收缓存，设为3000为佳。
	int i,j;
	
	int *run=(int*)param;//线程启动，退出控制。
    int ind=0;
	
	while((*run)&0x0f)
	{
		if((reclen=VCI_Receive(VCI_USBCAN2,0,ind,rec,3000,100))>0)//调用接收函数，如果有数据，进行数据处理显示。
		{
			for(j=0;j<reclen;j++)
			{
				uint can_id = rec[j].ID;
				if(rec[j].Data[0] == 1 && rec[j].Data[1]==2 && rec[j].Data[2] == 3 && rec[j].Data[3] == 4){ // 101
					int dianliang=rec[j].Data[4];
					float kongzhiquanxian;
					//原来data[5]==3，kongzhiquanxian是1时，tcp状态是2，表示自动
					//kongzhiquanxian在关机时候或者脱离一定距离时会自动改为1，也就是自动
					//data[5]==2的时候是手动状态下是0
					int kongzhi = rec[j].Data[5];
					// cout<< "danpianjifankui::::"<< kongzhi << endl;
					if(rec[j].Data[5] == 3 ) {kongzhiquanxian=1;}
					else	{kongzhiquanxian=0;}
					if(rec[j].Data[6] == 1){//如果为1，表示agv载货
						fuwei=1;
						ros::param::set("zaihuo",fuwei);
						//ros::param::set("renwu_finish",2);
					}else if(fuwei_last != fuwei){
						fuwei=2;
						fuwei_last=fuwei;
						ros::param::set("zaihuo",fuwei);
						//ros::param::set("renwu_finish",3);
					}

					// cout<<"dianliang  "<<dianliang<<endl;
					ros::param::set("dianliang",dianliang);
					ros::param::set("kongzhiquanxian",kongzhiquanxian);

				}else if(can_id == 547){
					uint32_t lidar_right_1 = rec[j].Data[4];//右侧雷达低位
					uint32_t lidar_right_2 = rec[j].Data[5];//右侧雷达高位
					uint32_t lidar_left_1 = rec[j].Data[6];//左侧前方雷达低位
					uint32_t lidar_left_2 = rec[j].Data[7];//以车为方向，左侧前方雷达高位
					string lidar_right_1_str = Number2HexStr(lidar_right_1);
					string lidar_right_2_str = Number2HexStr(lidar_right_2);
					string lidar_left_1_str = Number2HexStr(lidar_left_1);
					string lidar_left_2_str = Number2HexStr(lidar_left_2);
					if(lidar_right_1 != 0){
						string lidar_right_final = lidar_right_2_str + lidar_right_1_str;
						long long right_final = stoll(lidar_right_final,nullptr,16);
						// cout<<"右侧雷达:::"<<right_final<<endl;
						if(right_final < 600){
							//ros::param::set("rader_right_stop",1);
						}else{
							ros::param::set("rader_right_stop",0);
						}
					}
					if(lidar_left_1 != 0){
						string lidar_left_final = lidar_left_2_str + lidar_left_1_str;
						long long left_final = stoll(lidar_left_final,nullptr,16);
						// cout<<"左侧雷达:::"<<left_final<<endl;
						if(left_final < 600){
							//ros::param::set("rader_left_stop",1);
						}else{
							ros::param::set("rader_left_stop",0);
						}
					}
					uint32_t weight_1 = rec[j].Data[0];//称重的最低位
					uint32_t weight_2 = rec[j].Data[1];
					uint32_t weight_3 = rec[j].Data[2];
					uint32_t weight_4 = rec[j].Data[3];//称重的最高位
					if(weight_4 != 0xff){
						string str_weight_1 = Number2HexStr(weight_1);
						string str_weight_2 = Number2HexStr(weight_2);
						string str_weight_3 = Number2HexStr(weight_3);
						string str_weight_4 = Number2HexStr(weight_4);
						string str_weight__final = str_weight_4 + str_weight_3 + str_weight_2 + str_weight_1;
						long long w_final = stoll(str_weight__final,nullptr,16);
						float float_weight = std::stof(std::to_string(w_final));
						// cout<<"重量:::"<<float_weight<<endl;
						ros::param::set("zaihuozhongliang",float_weight);
					}else{
						float weight_null = 0;
						// cout<<"重量:::"<<weight_null<<endl;
						ros::param::set("zaihuozhongliang",weight_null);
					}
				}else if (can_id == 42532864){
					pos_sen.pos_rb = -(double)(rec[j].Data[7]-90)/57.3;
					pos_sen.pos_lb = (double)(rec[j].Data[5]-90)/57.3;//
					pos_sen.pos_rf = (double)(rec[j].Data[6]-90)/57.3;
					pos_sen.pos_lf = -(double)(rec[j].Data[4]-90)/57.3;//

					double pos_lf = (double)(rec[j].Data[4]-90)/57.3;//
					double pos_lb = (double)(rec[j].Data[5]-90)/57.3;//
					double pos_rf = (double)(rec[j].Data[6]-90)/57.3;
					double pos_rb = (double)(rec[j].Data[7]-90)/57.3;
					
					speed_sen.speed_lf=(double)(rec[j].Data[0]-100)/100;
					speed_sen.speed_lb=-(double)(rec[j].Data[1]-100)/100;
					speed_sen.speed_rf=(double)(rec[j].Data[2]-100)/100;
					speed_sen.speed_rb=-(double)(rec[j].Data[3]-100)/100;
				}	

				double a=abs(pos_sen.pos_lb-pos_sen.pos_lf)+abs(pos_sen.pos_lb-pos_sen.pos_rb)+abs(pos_sen.pos_lb-pos_sen.pos_rf);
				double b=(abs(pos_sen.pos_lf+1.57)+abs(pos_sen.pos_lb-1.57)+abs(pos_sen.pos_rb+1.57)+abs(pos_sen.pos_rf-1.57))/4;
				if(a <0.1 ){ 
					deal_back(speed_sen,pos_sen);
				}else if(b<0.1){
					deal_back2(speed_sen,pos_sen);
				}
				else{
					deal_back3(speed_sen,pos_sen);
				}
				// printf("RX");
				// printf("\n");
				odom.header.frame_id = "odom";
				current_time = ros::Time::now();
				odom.header.stamp = current_time;
				odom.pose.pose.position.y = posotion.pos_x;
				odom.pose.pose.position.x = posotion.pos_y;
				odom.pose.pose.position.z = posotion.angle;

				odom.twist.twist.linear.x =  vx;
				odom.twist.twist.linear.y =  vy;
				odom.twist.twist.linear.z=0;

				odom.twist.twist.angular.z = w;
				odom.twist.twist.angular.x=0;
				odom.twist.twist.angular.y=0;
				// odom_pub.publish(odom);
			}
		}
		ind=!ind;//变换通道号，以便下次读取另一通道，交替读取。		
	}
	printf("run thread exit\n");//退出接收线程	
	pthread_exit(0);
}

void callback(const geometry_msgs::Twist& cmd_vel)
{
	 vx = cmd_vel.linear.x
	, vy =  cmd_vel.linear.y
	, w = cmd_vel.angular.z;
	//规则： Vx不能给负数，有三种情况，1、舵论行使，此时w为0
	// 2自旋， 此时w有值，xvy为0
	// 3阿克曼 此时vy为0 ，vx，w有值，此后若有倒车控制，则可单写一个规则，if(vy == 9999) 
	//   vx =-0.1
	// , vy = 0.099998888
	// , w =0;

	VCI_CAN_OBJ send[1];
	//需要发送的帧，结构体设置
	send[0].ID=0x029A;
	send[0].SendType=0;
	send[0].RemoteFlag=0;
	send[0].ExternFlag=0;
	send[0].DataLen=8;

	// cout<<"-----------zhengchangxingshi-----------"<<endl;
	if( w == 0  ){//舵轮平移
	// cout<<"111111111111111111111111111111"<<endl;
		range_precote(vx, 0.2, -0.2);
		range_precote(vy, 0.2, -0.2);
		range_precote(w, 0.25, -0.25);
		deal1(vx, vy, w, myvtospeed_);
	}else if(w!=0 && vx == 0 && vy == 0){//自旋
	// cout<<"222222222222222222222222222222"<<endl;
		range_precote(vx, 0.5 ,-0.5);
		range_precote(vy, 0.5, -0.5);
		range_precote(w, 0.25, -0.25);
		deal2(vx, vy, w, myvtospeed_);
	}else{//akm
	// cout<<"3333333333333333333333333333333"<<endl;
		range_precote(vx, 0.5 ,-0.5);
		range_precote(vy, 0.5, -0.5);
		range_precote(w, 0.25, -0.25);
		deal3(vx, vy, w, myvtospeed_);
	}

	send[0].Data[0] = (Alf*57.3+90);//rf 0
	send[0].Data[1] = (Alb*57.3+90);//lf
	send[0].Data[2] = (Arf*57.3+90);//br
	send[0].Data[3] = (Arb*57.3+90);//bl
	
	send[0].Data[4] = myvtospeed_.speed_Vlf; // 100
	send[0].Data[5] = myvtospeed_.speed_Vlb;
	send[0].Data[6] = myvtospeed_.speed_Vrf;
	send[0].Data[7] = myvtospeed_.speed_Vrb;//

	float lb_=0,rb_=0,lf_=0,rf_=0;
	float err_=35;
	lb_ = abs((Alb * 57.3) - pos_sen.pos_lb * 57.3);
	rb_ = abs((Arb * 57.3) - pos_sen.pos_rb * 57.3);
	lf_ = abs((Alf * 57.3) - pos_sen.pos_lf * 57.3);
	rf_ = abs((Arf * 57.3) - pos_sen.pos_rf * 57.3);

	if(lb_>err_ || rb_>err_ || rf_>err_ || lf_>err_){
		send[0].Data[4] = 100; // 100
		send[0].Data[5] = 100; // 100
		send[0].Data[6] = 100; // 100
		send[0].Data[7] = 100; // 100
		cout << "wati_turn!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << endl;
	}

	if(abs(vx) <0.01 && abs(vy)<0.01 &&abs(w)<0.01){
	send[0].Data[0] = (0*57.3+90);//rf 90
	send[0].Data[1] = (0*57.3+90);//lf
	send[0].Data[2] = (0*57.3+90);//br
	send[0].Data[3] = (0*57.3+90);//bl
	}

	if(vy ==0.099998888){//
		send[0].Data[4] = 90; // 100
		send[0].Data[5] = 90; // 100
		send[0].Data[6] = 110; // 100
		send[0].Data[7] = 110; // 100
		send[0].Data[0] = (0*57.3+90);//rf 90
		send[0].Data[1] = (0*57.3+90);//lf
		send[0].Data[2] = (0*57.3+90);//br
		send[0].Data[3] = (0*57.3+90);//bl
	}

	if (VCI_Transmit(VCI_USBCAN2, 0, 1, send, 1) == 1)
	{
		printf(" TX ");
		printf("\n");	
	}
} 

int main(int argc, char  *argv[])
{
	ros::init(argc,argv,"can_sub_and_pub");
	ros::NodeHandle nh;
	ros::param::set("fuwei",0);
	ros::param::set("zaihuozhongliang",0);
	ros::param::set("rader_left_stop",0);
	ros::param::set("rader_right_stop",0);

	if(VCI_OpenDevice(VCI_USBCAN2,0,0)==1)//打开设备
	{
		printf(">>open deivce success!\n");//打开设备成功
	}else
	{
		printf(">>open deivce error!\n");
		exit(1);
	}

	//初始化参数，严格参数二次开发函数库说明书。
	VCI_INIT_CONFIG config;
	config.AccCode=0;
	config.AccMask=0xFFFFFFFF;
	config.Filter=1;//接收所有帧
	config.Timing0=0x00;/*波特率125 Kbps  0x03  0x1C*/
	config.Timing1=0x1C;
	config.Mode=0;//正常模式		
	
	if(VCI_InitCAN(VCI_USBCAN2,0,0,&config)!=1)
	{
		printf(">>Init CAN1 error\n");
		VCI_CloseDevice(VCI_USBCAN2,0);
	}

	if(VCI_StartCAN(VCI_USBCAN2,0,0)!=1)
	{
		printf(">>Start CAN1 error\n");
		VCI_CloseDevice(VCI_USBCAN2,0);

	}

	if(VCI_InitCAN(VCI_USBCAN2,0,1,&config)!=1)
	{
		printf(">>Init can2 error\n");
		VCI_CloseDevice(VCI_USBCAN2,0);

	}
	if(VCI_StartCAN(VCI_USBCAN2,0,1)!=1)
	{
		printf(">>Start can2 error\n");
		VCI_CloseDevice(VCI_USBCAN2,0);

	}
	int m_run0=1;
	pthread_t threadid;
	int ret;
	ret=pthread_create(&threadid,NULL,receive_func,&m_run0);
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// odom_pub = nh.advertise<nav_msgs::Odometry>("odom_by", 1);
	ros::Subscriber sub = nh.subscribe("cmd_vel", 1, callback);
	 ros::Timer timer1 = nh. createTimer ( ros::Duration ( 1 ), timer_cb1);
	// ros::Timer timer2 = nh. createTimer ( ros::Duration ( 5 ), timer_cb2);

	while(ros::ok()){
		usleep(200000);//延时单位us，这里设置 10 000 000=10s    10s后关闭接收线程，并退出主程序。  20000
		
		ros::spinOnce();
	}


	usleep(1000000);//延时单位us，这里设置 10 000 000=10s    10s后关闭接收线程，并退出主程序。
	m_run0=0;//线程关闭指令。
	pthread_join(threadid,NULL);//等待线程关闭。
	usleep(100000);//延时100ms。
	VCI_ResetCAN(VCI_USBCAN2, 0, 0);//复位CAN1通道。
	usleep(100000);//延时100ms。
	VCI_ResetCAN(VCI_USBCAN2, 0, 1);//复位CAN2通道。
	usleep(100000);//延时100ms。
	VCI_CloseDevice(VCI_USBCAN2,0);//关闭设备。
	//除收发函数外，其它的函数调用前后，最好加个毫秒级的延时，即不影响程序的运行，又可以让USBCAN设备有充分的时间处理指令。
	//goto ext;
}
