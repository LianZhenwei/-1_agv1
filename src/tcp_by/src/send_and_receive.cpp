#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <fcntl.h>
#include <sys/shm.h>
#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include <fstream>
#include "math.h"
#include "tcp_by/task.h"
#include <iostream>  
#include "tf/tf.h"
using namespace std;
int tilong_;
int renwu_finish=0;
int liushui_int;
string liushui_globle="0000";
vector<string> map_msg;
int cnt = 0; 
int con = 0; 
int dianliang=0;



struct sub_all{
	string id_c;
	string gongneng;
	string len_c;
	string over_flag_c;
	bool chaxun;
	string id_r;
	string gongneng_r;
	string len_r;
	string renwuliushui_r;
	string renwuleixing_r;
	string weizhi_r;
	string over_flag_r;
	bool renwu;
	string gengxinzhuangtai;
};
struct send_all{
	string id_c;
	string gongneng_c;
	string len_c;
	string state_shebei_c;
	string state_gongzuo_c;
	string shengjiang_c;
	string zaihuo_c;
	string tilong_c;
	string renwu_finish_c;
	string dianliang_c;
	string renwuliushui_c;
	string guzhang_c;
	string weizhi_c;
	string changjingid;
	string over_c;
	string louceng;
	string gengxinzhuangtai;
	string xingshilicheng;
	string xingshisudu;
	string zaihuozhongliang;

	string id_r;
	string gongneng_r;
	string len_r;
	string renwuliushui_r;
	string renwuzhuangtai_r;
	string over_r;

	string road_id;
	vector<pair<string,string>> point;
	vector<string> beh;
	vector<string>  pre;
};
#define    MYPORT     8000   //端口号
#define    BUF_SIZE   100  //数据缓冲区最大长度
 
char* SERVER_IP = "192.168.3.3";


int socket_cli = socket(AF_INET, SOCK_STREAM, 0);

void string2hexString(char* input, char* output)
{
    int loop;
    int i; 
    
    i=0;
    loop=0;
    
    while(input[loop] != '\0')
    {
        sprintf((char*)(output+i),"%02X", input[loop]);
        loop+=1;
        i+=2;
		// cout<<i<<endl;
    }
    //insert NULL at the end of the output string
    output[i++] = '\0';
}
std::string HexToString(const char* hex, size_t size) {  

    std::stringstream ss;  

    for (size_t i = 0; i < size; ++i) {  

       ss << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(hex[i]); 

    }  

    return ss.str();  

}
std::string floatToString(float value) {  

    std::ostringstream stream;  

    stream << std::fixed << std::setprecision(2) << value;  

    return stream.str();  

}
std::string hexToString1(const std::string& hex) {  
    std::stringstream ss;  
    ss << std::hex;  
    for (char c : hex) {  
        ss >> c;  
    }  
    std::string result;  
	string c;
    while (ss >> c) {  
        result += c;  
    }  
    return result;  
}  
std::string padZero(std::string str, int targetLength) {  

    while (str.length() < targetLength) {  
        str = "0" + str;  
    }  

    return str;  

}

void timer_cb1 ( const ros::TimerEvent & )
{
	cout<<"111111111111111111111111111111111111111111111111111111"<<endl;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "tcp_client_node");
    ros::NodeHandle n;
   	tcp_by::task task_;
    sub_all sub;
    send_all send_msg;
    /*
	 *@fuc: socket()创建套节字
	 *
	 */
	if(socket_cli < 0)
	{
		std::cout << "socket() error\n";
		return -1;
	}
	
	/*
	 *@fuc: 服务器端IP4地址信息,struct关键字可不写
	 *@fuc: 初始化sever地址信息   
	 */
	struct sockaddr_in sev_addr;  
	memset(&sev_addr, 0, sizeof(sev_addr));
	sev_addr.sin_family      = AF_INET;
	sev_addr.sin_port        = htons(MYPORT);
	sev_addr.sin_addr.s_addr = inet_addr(SERVER_IP);
	std::cout << "connecting..." << std::endl;
	/*
	 *@fuc: 使用connect()函数来配置套节字,建立一个与TCP服务器的连接
	 */
	con = connect(socket_cli, (struct sockaddr*) &sev_addr, sizeof(sev_addr));
	if( con < 0)
	{
		std::cout << "connect error" << std::endl;
		return -1;
	}
	else
		std::cout << "connected successfullly!" << std::endl;


	ros::Timer timer1 = n. createTimer ( ros::Duration ( 1 ),timer_cb1);
	while(ros::ok)
	{
		char recvbuf[BUF_SIZE];
		// map_updata();
		/*
		 *@fuc: 使用recv()函数来接收服务器发送的消息
		 */
		cnt = (int)recv(socket_cli, recvbuf, sizeof(recvbuf), 0);
		if(cnt>0)
		{
		//declare output string with double size of input string
		//because each character of input string will be converted
		//in 2 bytes	
		ros::Rate rate1(1);	
		string str="00";
		str=HexToString(recvbuf,sizeof(recvbuf));
		cout<<str.length()<<endl;
		cout << "接收到的数据为  " << str << endl;
		sub.gongneng=str.substr(8,2);
		if(sub.gongneng=="a1"){
				cout<<"---------------------------小车状态查询---------------------------"<<endl;
				send_msg.id_c = "02";//小车id
				send_msg.gongneng_c = "A1";//功能
				send_msg.len_c = "41";//数据长度
				//控制权限(设备状态)
				int kongzhiquanxian=0;
				ros::param::get("kongzhiquanxian",kongzhiquanxian);
				if(kongzhiquanxian == 1){
					send_msg.state_shebei_c = "02";//自动
				}else{
					send_msg.state_shebei_c = "01";
				}
				//工作状态
				int work_state=0;
				ros::param::get("work_state",work_state);
				if(work_state == 1){
					send_msg.state_gongzuo_c = "01";
					cout<<"空闲"<<endl;
				}else if(work_state == 2){
					cout<<"忙碌"<<endl;
					send_msg.state_gongzuo_c = "02";
				}else{
					send_msg.state_gongzuo_c = "01";	
				}
				//升降状态
				send_msg.shengjiang_c = "02";
				//载货状态
				int zaihuo_state=2;
				ros::param::get("zaihuo",zaihuo_state);
				if(zaihuo_state==1){
					send_msg.zaihuo_c="01";
					cout<<"有载"<<endl;
				}else{
					send_msg.zaihuo_c="02";
					cout<<"无载"<<endl;
				}
				//任务完成状态
				ros::param::get("renwu_finish",renwu_finish);
				send_msg.renwu_finish_c = "00";
				if(renwu_finish==0 && work_state==1){
					send_msg.renwu_finish_c = "00";
				}else if(renwu_finish==1){
					send_msg.renwu_finish_c = "01";
					//已移动至目的经纬度
				}else if(renwu_finish==2){
					send_msg.renwu_finish_c = "02";
				}else if(renwu_finish==3){
					send_msg.renwu_finish_c = "03";
				}else if(renwu_finish==4){
					send_msg.renwu_finish_c = "04";
				}else if(renwu_finish==5){
					send_msg.renwu_finish_c = "05";
				}else if(renwu_finish==6){
					send_msg.renwu_finish_c = "06";
				//已到梯笼前定位成功
				}else if(renwu_finish==7){
					send_msg.renwu_finish_c = "07";
				//已进梯笼
				}else if(renwu_finish==8){
					send_msg.renwu_finish_c = "08";
				//已出梯笼
				}else if(renwu_finish==9){
					send_msg.renwu_finish_c = "09";
				}else if(renwu_finish==10){
					send_msg.renwu_finish_c = "10";
				}else if(renwu_finish==11){
					send_msg.renwu_finish_c = "11";
				}else if(renwu_finish==12){
					send_msg.renwu_finish_c = "12";
				}else if(renwu_finish==13){
					send_msg.renwu_finish_c = "13";
				}else if(renwu_finish==99){
					send_msg.renwu_finish_c = "99";
				}

				//当前电量
				ros::param::get("dianliang",dianliang);
				string dianchi_str = std::to_string(dianliang);
				dianchi_str=padZero(dianchi_str,2);
				cout<<"tcp------dianliang----"<<dianchi_str<<endl;
				send_msg.dianliang_c = dianchi_str;
				//任务流水
				string liushui="0000";
				ros::param::get("liushui", liushui);
				send_msg.renwuliushui_c = liushui;
				//故障代码
				send_msg.guzhang_c = "00";
				//场景id
				send_msg.changjingid="01";
				//当前楼层
				int floor=1;
				string floor_str="01";
				ros::param::get("floor_py",floor);
				if(floor == 1){
					floor_str="01";
				}
				if(floor == 2){
					floor_str="02";
				}
				if(floor == 3){
					floor_str="03";
				}
				if(floor == 4){
					floor_str="04";
				}
				if(floor == 5){
					floor_str="05";
				}
				if(floor == 6){
					floor_str="06";
				}
				if(floor == 7){
					floor_str="07";
				}
				if(floor == 8){
					floor_str="08";
				}
				if(floor == 9){
					floor_str="09";
				}
				if(floor == 10){
					floor_str="10";
				}
				if(floor == 11){
					floor_str="11";
				}
				if(floor == 12){
					floor_str="12";
				}
				if(floor == 13){
					floor_str="13";
				}
				if(floor == 11){
					floor_str="11";
				}
				if(floor == 14){
					floor_str="14";
				}
				if(floor == 15){
					floor_str="15";
				}
				if(floor == 16){
					floor_str="16";
				}
				if(floor == 17){
					floor_str="17";
				}
				if(floor == 18){
					floor_str="18";
				}
				if(floor == 19){
					floor_str="19";
				}
				if(floor == 20){
					floor_str="20";
				}
				if(floor == 21){
					floor_str="21";
				}
				if(floor == 22){
					floor_str="22";
				}
				if(floor == 24){
					floor_str="24";
				}
				if(floor == 26){
					floor_str="26";
				}
				if(floor == 28){
					floor_str="28";
				}
				if(floor == 30){
					floor_str="30";
				}
				if(floor == 99){
					floor_str="01";
				}
				send_msg.louceng=floor_str;
				//位置
				float x_=0,y_=0,theta_=0;
				ros::param::get("x",x_);
				ros::param::get("y",y_);
				ros::param::get("theta",theta_);
				string x= floatToString(x_);
				string y= floatToString(y_);
				string theta=floatToString(theta_);
				x=padZero(x, 12);
				y=padZero(y, 12);
				theta=padZero(theta, 12);
				send_msg.weizhi_c = x+","+y+","+theta;
				//行使里程
				float all_distance_ = 0;
				ros::param::get("total_distance_",all_distance_);
				all_distance_ = roundf(all_distance_ * 10)/10;
				cout<<"all_dis"<<all_distance_<<endl;
				string all_distance = floatToString(all_distance_);
				all_distance=padZero(all_distance,12);
				send_msg.xingshilicheng = all_distance;
				//行驶速度
				float veh_distance_ = 0;
				ros::param::get("veh_distance",veh_distance_);
				veh_distance_ = roundf(veh_distance_ * 10)/10;
				cout<<"veh:"<<veh_distance_<<endl;
				string veh_distance = floatToString(veh_distance_);
				veh_distance=padZero(veh_distance,4);
				send_msg.xingshisudu = veh_distance;
				//载货量
				float zaihuozhongliang_ = 0;
				ros::param::get("zaihuozhongliang",zaihuozhongliang_);
				string zaihuozhongliang_str = floatToString(zaihuozhongliang_);
				zaihuozhongliang_str=padZero(zaihuozhongliang_str,9);
				send_msg.zaihuozhongliang = zaihuozhongliang_str;
				//完成位
				send_msg. over_c="FF";
				string send_msg1;
				send_msg1.append(send_msg. id_c);
				send_msg1.append(send_msg. gongneng_c);
				send_msg1.append(send_msg. len_c);
				send_msg1.append(send_msg. state_shebei_c);
				send_msg1.append(send_msg.state_gongzuo_c);
				send_msg1.append(send_msg. shengjiang_c);
				send_msg1.append(send_msg. zaihuo_c);
				send_msg1.append(send_msg.renwu_finish_c );
				send_msg1.append(send_msg. dianliang_c);
				send_msg1.append(send_msg. renwuliushui_c);
				send_msg1.append(send_msg. guzhang_c);
				send_msg1.append(send_msg. changjingid);
				send_msg1.append(send_msg. louceng);
				send_msg1.append(send_msg. weizhi_c);
				send_msg1.append(send_msg. xingshilicheng);
				send_msg1.append(send_msg. xingshisudu);
				send_msg1.append(send_msg. zaihuozhongliang);
				send_msg1.append(send_msg. over_c);
		
				cout<<"回复的内容为 "<<send_msg1<<endl;
				const char *q;
				q = send_msg1.c_str();
				send(socket_cli, q,send_msg1.length(), 0);
			}
		rate1.sleep();
		ros::spinOnce();
		}
		else{
			close(socket_cli);
			ros::Duration du(5);//持续10秒钟,参数是double类型的，以秒为单位
			du.sleep();//按照指定的持续时间休眠
			int socket_cli_2 = socket(AF_INET, SOCK_STREAM, 0);
			if( connect(socket_cli_2, (struct sockaddr*) &sev_addr, sizeof(sev_addr))< 0)
			{
				std::cout << "connect error" << std::endl;
				cout<<"socket_cli"<<endl;
				
			}
			else
			std::cout << "connected successfullly!" << std::endl;
		}
	}
	
	/*
	 *@fuc: 关闭连接
	 */
	close(socket_cli);
	return 0;
}

