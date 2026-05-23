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
#define GPS
using namespace std;
int tilong_;
int liushui_int;
string liushui_globle="0000";
vector<string> map_msg;
int map_flag_start=0;
int map_flag_end=0;
int map_first_flag=0;
int map_over_flag=0;
int cnt_2 = 0;

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
	string floor;
	string jiting;
	string zaihuo;
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
	string dianliang_c;
	string renwuliushui_c;
	string guzhang_c;
	string weizhi_c;
	string changjingid;
	string over_c;
	string louceng;
	string gengxinzhuangtai;

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
#define    MYPORT     8001   //端口号
#define    BUF_SIZE   100  //数据缓冲区最大长度
 
char* SERVER_IP = "192.168.3.3";

struct Point {
    double x;
    double y;
    double l;
    double r;
    double s;
    double theta;
};
struct Road
{
    int id;
    std::vector<int> pre;
    std::vector<int> beh;
    double distance;
    std::vector<Point> road_points;
};
vector<Road> roads;
int socket_cli = socket(AF_INET, SOCK_STREAM, 0);


std::string toUpperCase(const std::string &input) {

    std::string output = input; // 创建一个与input相同内容的副本，以避免修改原始字符串

    std::transform(output.begin(), output.end(), output.begin(), 

                   [](unsigned char c){ return std::toupper(c); });

    return output;

}
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

        ss << std::hex << std::setw(2) << std::setfill('0') << (int)hex[i];  

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

void map_updata(){
		ros::Rate rate(1);
		rate.sleep();
	    string roadMap_path = "/home/agv/agv/data/slam8.txt";//地图录制位置road_path_by_gps.txt    
    	roads = load_direction_Road(roadMap_path);
		send_all send_map;
		send_map.id_c="01";
		send_map.gongneng_c = "A3";
		string send_msg1;
		send_msg1 = send_map.id_c +
					send_map.gongneng_c +
					"7777"+"\n";
		for(int j=0;j<roads.size();j++){

			send_map.point.clear();
			send_map.pre.clear();
			send_map.beh.clear();

			char t[256];
			sprintf(t,"%d",roads[j].id);
			send_map.road_id ="road "+(string)t;
			send_msg1+=send_map.road_id;

			for(int i=0;i<roads[j].road_points.size();i++){
				char x[256];
				roads[j].road_points[i].x=(int)(roads[j].road_points[i].x*100);
				roads[j].road_points[i].x=(float)roads[j].road_points[i].x/100;

				roads[j].road_points[i].y=(int)(roads[j].road_points[i].y*100);
				roads[j].road_points[i].y=(float)roads[j].road_points[i].y/100;

				sprintf(x, "%f", roads[j].road_points[i].x);
				char y[256];
				sprintf(y, "%f", roads[j].road_points[i].y);
				string x_s=x,y_s=y;
				pair<string,string> xy;
				x_s = x_s.substr(0, x_s.length() - 4);  
				y_s = y_s.substr(0, y_s.length() - 4);  
				xy.first=x_s;
				xy.second=y_s;

				send_map.point.push_back(xy);
			}

			for(int i=0;i<roads[j].beh.size();i++){
				char b[256];
				sprintf(b, "%d", roads[j].beh[i]);
				send_map.beh.push_back(b);
			}

			for(int i=0;i<roads[j].pre.size();i++){
				char p[256];
				sprintf(p, "%d", roads[j].pre[i]);
				send_map.pre.push_back(p);
			}
			for (int i = 0; i < send_map.point.size(); i++)
			{
				send_msg1 += "\n";
				send_msg1 += send_map.point[i].first;
				send_msg1 += " ";
				send_msg1 += send_map.point[i].second;
			}
			send_msg1 += "\n";

			send_msg1 += "pre ";
			for (int i = 0; i < send_map.pre.size(); i++)
			{
				send_msg1 += send_map.pre[i];
			}
			send_msg1 += "\n";
			send_msg1 += "beh ";
			for (int i = 0; i < send_map.beh.size(); i++)
			{
				send_msg1 += send_map.beh[i];
			}
			send_msg1 += "\n";
		}
		send_msg1+="8888";
		send_msg1+="FF";

		int size_by = send_msg1.length()-6;
		cout<<"size   "<<size_by<<endl;
		char b[256];
		sprintf(b, "%d", size_by);
		string b_str=b;
		while (b_str.length() < 8) {  
        	b_str.insert(0, "0");  
    	} 
		send_msg1.insert(4,b_str);// 在指定位置插入1个字符c  
		//cout<<send_msg1<<endl;

	const char *q;
	q = send_msg1.c_str();
	// cout<<"///"<<q<<endl;
	send(socket_cli, q, send_msg1.length(), 0);
}
void map_get(char recvbuf[]){
string map_str = recvbuf;
		int start=map_str.find("7777");
		int end=map_str.find("8888");

		if(start != (-1) ){
			map_flag_start=1;
			map_first_flag=1;
		}else if(map_first_flag){
			map_flag_start=2;
		}else{
			map_flag_start=0;
		}

		if(end!= std::string::npos){
			map_flag_end=1;
		}else{
			map_flag_end=0;
		}
	
		if(map_flag_start == 1 && map_flag_end==0){
			string temp=recvbuf;
			temp=temp.substr(start+4,temp.length());
			 map_msg.push_back(temp);
		}
		if(map_flag_start == 1 && map_flag_end==1){
			string temp=recvbuf;
			temp=temp.substr(start+4,end);
			map_msg.push_back(temp);
		}
		if(map_flag_start == 2 && map_flag_end==0){
			string temp=recvbuf;
			temp=temp.substr(0,temp.length());
			map_msg.push_back(temp);
		}
		if(map_flag_start == 2 && map_flag_end==1){
			string temp=recvbuf;
			temp=temp.substr(0,end);
			map_msg.push_back(temp);
			map_over_flag=1;
		}
		if(map_over_flag){
			string map_end;
			for(int i=0;i<map_msg.size();i++){
				map_end=map_end+map_msg[i];
			}
			cout << "地图数据为  " << map_end << endl;
			string roadMap_path = "/home/agv/agv/data/slam90.txt"; // 地图录制位置road_path_by_gps.txt
			fstream outFile(roadMap_path, std::ios::app);
			outFile << map_end;
			cout << "写入成功";
			outFile.close();
				cout<<"---------------------------小车接受地图---------------------------"<<endl;
				send_all sub_map;
				sub_map.id_c="01";
				sub_map.gongneng_c = "A4";
				string send_msg1;
				send_msg1 = sub_map.id_c +
							sub_map.gongneng_c +
							"FFFFFFFF"+
							"01"+
							"FF";

				int size_by = send_msg1.length();
				char b[256];
				sprintf(b, "%d", size_by);


				const char *q;
				q = send_msg1.c_str();
				send(socket_cli, q, send_msg1.length(), 0);
				 map_flag_start=0;
				 map_flag_end=0;
				 map_first_flag=0;
				 map_over_flag=0;
		}
}
// 函数用于删除连续的六个 'f'
void remove_consecutive_f(std::string& str) {

    for (size_t i = 0; i <= str.size() - 6; ++i) {

        // 检查是否找到了连续六个 'f'

        if (str.substr(i, 6) == "ffffff") {

            // 如果找到，从字符串中删除它们

            str.erase(i, 6);

            // 注意，由于erase会改变原字符串的大小，因此i不变会导致下一次检查时的位置错误

            // 因此，在删除字符之后，需要回溯一位以检查当前位置及其后续位置

            --i;

        }

    }

}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "tcp_client_node_2");
    ros::NodeHandle n;
    ros::Publisher point_pub = n.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 10);
	ros::Publisher task_pub = n.advertise<tcp_by::task>("task_by", 10);
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
	if(connect(socket_cli, (struct sockaddr*) &sev_addr, sizeof(sev_addr)) < 0)
	{ 
		std::cout << "connect error" << std::endl;
		return -1;
	}
	else
		std::cout << "connected successfullly!" << std::endl;
		
	while(ros::ok)
	{
		int floor=0;
		ros::param::get("floor",floor);
		char recvbuf[BUF_SIZE];
		// map_updata();
		/*
		 *@fuc: 使用recv()函数来接收服务器发送的消息
		 */
		cout<<"--------------------------jinru2--------------------------"<<endl;
		cnt_2 = (int)recv(socket_cli, recvbuf, sizeof(recvbuf), 0);
		cout<<"-------------------------jinru3----------------------------"<<endl;
		//declare output string with double size of input string
		//because each character of input string will be converted
		//in 2 bytes	
		if(cnt_2>0){
		ros::Rate rate1(10);	
		string str;
		str=HexToString(recvbuf,sizeof(recvbuf));
		remove_consecutive_f(str);
		cout << "len<1000接收到的数据为" << str << endl;
		sub.gongneng=str.substr(2,2);
			if(sub.gongneng=="a2"){
				cout<<"---------------------------下发任务---------------------------"<<endl;
			
				sub.gongneng=str.substr(2,2);
				sub.renwuleixing_r=str.substr(10,2);
				string liushui=str.substr(6,4);
				liushui=toUpperCase(liushui);
			
				ros::param::set("liushui", liushui.c_str());

				liushui_int = stoi(liushui,0,16);
				liushui_globle = to_string(liushui_int);
				cout<<"-------任务流水是---------"<<liushui_globle<<endl;
				
				if(sub.renwuleixing_r=="01"){
					cout<<"任务类型是移动到指定位置"<<endl;
					ros::param::set("renwu_finish",99);
					ros::param::set("work_state",2);
		  			ros::param::set("task",1);
                    geometry_msgs::PoseStamped p;

					string x = str.substr(12, 24);
					std::string hexString = x ; // 输入的16进制字符串
					int length = hexString.length();
					char* buffer = new char[length / 2 + 1]; // 创建足够大小的字符数组来存放结果
					for (int i = 0; i < length; i += 2) {
					sscanf(hexString.substr(i, 2).c_str(), "%x", &buffer[i/2]); // 逐两位读取并转换成十六进制值
					}
					buffer[length / 2] = '\0'; // 在最后添加空字符作为字符串的结尾标志

					x=buffer;
					delete[] buffer; // 记得释放内存
					std::cout << "转换后的字符串为: " << x << std::endl;

					string y = str.substr(38, 24);
					std::string hexString_y = y ; // 输入的16进制字符串
					int length_y = hexString_y.length();
					char* buffer_y = new char[length_y / 2 + 1]; // 创建足够大小的字符数组来存放结果
					for (int i = 0; i < length_y; i += 2) {
					sscanf(hexString_y.substr(i, 2).c_str(), "%x", &buffer_y[i/2]); // 逐两位读取并转换成十六进制值
					}
					buffer_y[length_y / 2] = '\0'; // 在最后添加空字符作为字符串的结尾标志

				
					y=buffer_y;
					delete[] buffer_y; // 记得释放内存

					string theta = str.substr(64, 24);
					std::string hexString_theta = theta ; // 输入的16进制字符串
					int length_theta = hexString_theta.length();
					char* buffer_theta = new char[length_theta / 2 + 1]; // 创建足够大小的字符数组来存放结果
					for (int i = 0; i < length_theta; i += 2) {
					sscanf(hexString_theta.substr(i, 2).c_str(), "%x", &buffer_theta[i/2]); // 逐两位读取并转换成十六进制值
					}
					buffer_theta[length_theta / 2] = '\0'; // 在最后添加空字符作为字符串的结尾标志


					theta=buffer_theta;
					delete[] buffer_theta; // 记得释放内存

					float num_x=0,num_y=0,num_theta=0;
				
				
					if(x.find('-')<x.length()){
						x = x.substr(x.find('-'));
					}
					

					if(y.find('-')<y.length()){
						y = y.substr(y.find('-'));
					}
					if(theta.find('-')<theta.length()){
						theta = theta.substr(theta.find('-'));
					}
					 num_x = std::stof(x); // 调用 stof() 进行转换
					 num_y = std::stof(y); // 调用 stof() 进行转换
					 num_theta = std::stof(theta); // 调用 stof() 进行转换
					std::cout << "转换后的字符串为: " << num_x << std::endl;
					std::cout << "转换后的字符串为: " << num_y << std::endl;
					std::cout << "转换后的字符串为: " << num_theta << std::endl;
				
				
					//只通过yaw, 即绕z的旋转角度计算四元数，用于平面小车。返回四元数
					tf::createQuaternionMsgFromYaw(num_theta).w;
					p.pose.position.x =num_x;
					p.pose.position.y = num_y;
					p.pose.position.z = 0;
					p.pose.orientation.x = 	tf::createQuaternionMsgFromYaw(num_theta).x;
					p.pose.orientation.y = 	tf::createQuaternionMsgFromYaw(num_theta).y;
					p.pose.orientation.z = 	tf::createQuaternionMsgFromYaw(num_theta).z;
					p.pose.orientation.w =	tf::createQuaternionMsgFromYaw(num_theta).w;
					for (int i = 0; i < 20; i++)
					{
						point_pub.publish(p);
					}
				}	
				if(sub.renwuleixing_r=="02"){
					ros::param::set("renwu_finish",2);
					ros::param::set("work_state",1);
					cout<<"任务类型是顶升，表示此时为有载"<<endl;
					ros::param::set("zaihuo",1);
				}
				if(sub.renwuleixing_r=="03"){
					ros::param::set("renwu_finish",3);
					ros::param::set("work_state",1);
					cout<<"任务类型是下降，表示此时为无载"<<endl;
					ros::param::set("zaihuo",2);
				}
				if(sub.renwuleixing_r=="06"){
					ros::param::set("renwu_finish",99);
					ros::param::set("work_state",2);
					cout<<"任务类型是在电梯前定位"<<endl;
                    ros::param::set("task",6);
					if(floor == 99  || floor == 1){
						ros::param::set("close_index",1);
					}else if(floor == 2){
						ros::param::set("close_index",2);
					}else if(floor == 3){
						ros::param::set("close_index",3);
					}else if(floor == 4){
						ros::param::set("close_index",4);
					}else if(floor == 5){
						ros::param::set("close_index",5);
					}else if(floor == 6){
						ros::param::set("close_index",6);
					}else if(floor == 7){
						ros::param::set("close_index",7);
					}else if(floor == 8){
						ros::param::set("close_index",8);
					}else if(floor == 10){
						ros::param::set("close_index",10);
					}else if(floor == 12){
						ros::param::set("close_index",12);
					}else if(floor == 14){
						ros::param::set("close_index",14);
					}else if(floor == 16){
						ros::param::set("close_index",16);
					}else if(floor == 18){
						ros::param::set("close_index",18);
					}else if(floor == 20){
						ros::param::set("close_index",20);
					}else if(floor == 22){
						ros::param::set("close_index",22);
					}else if(floor == 24){
						ros::param::set("close_index",24);
					}

				}
				if(sub.renwuleixing_r=="07"){
					ros::param::set("renwu_finish",99);
					cout<<"任务类型是进电梯"<<endl;
					ros::param::set("task",7);
					ros::param::set("work_state",2);
				}
                if(sub.renwuleixing_r=="08"){
					ros::param::set("renwu_finish",99);
					cout<<"任务类型是出电梯"<<endl;
					ros::param::set("task",8);
					ros::param::set("work_state",2);
                }
				if(sub.renwuleixing_r=="09")
				{
					ros::param::set("renwu_finish",99);
					cout << "任务类型是进码垛" << endl;
					ros::param::set("work_state",2);
					ros::param::set("task", 9);
					geometry_msgs::PoseStamped msg;
					msg.pose.position.x=12.61481;
					msg.pose.position.y=-2.874;
					msg.pose.position.z = 0;

					msg.pose.orientation.x = 0.017;
					msg.pose.orientation.y = -0.005;
					msg.pose.orientation.z = -0.680892;
					msg.pose.orientation.w = 0.732384;//-3.00544
					for (int i = 0; i < 20; i++)
					{
						point_pub.publish(msg);
					}
				}
				if(sub.renwuleixing_r=="0a")
				{
					ros::param::set("renwu_finish",99);
					cout << "任务类型是出码垛" << endl;
					ros::param::set("task", 10);
					ros::param::set("work_state",2);
				}
				if(sub.renwuleixing_r=="0b"){
					ros::param::set("renwu_finish",99);
					cout<<"任务类型是移动到电梯前，会伴随着发相应楼层电梯前的位置"<<endl;
					ros::param::set("work_state",2);
		  			ros::param::set("task",11);
					if(floor == 99  || floor == 1){
						ros::param::set("elevator",1);
					}else{
						ros::param::set("elevator",3);
					}
                    geometry_msgs::PoseStamped p;
					string x = str.substr(12, 24);
					std::string hexString = x ; // 输入的16进制字符串
					int length = hexString.length();
					char* buffer = new char[length / 2 + 1]; // 创建足够大小的字符数组来存放结果
					for (int i = 0; i < length; i += 2) {
					sscanf(hexString.substr(i, 2).c_str(), "%x", &buffer[i/2]); // 逐两位读取并转换成十六进制值
					}
					buffer[length / 2] = '\0'; // 在最后添加空字符作为字符串的结尾标志

					x=buffer;
					delete[] buffer; // 记得释放内存
					std::cout << "转换后的字符串为: " << x << std::endl;

					string y = str.substr(38, 24);
					std::string hexString_y = y ; // 输入的16进制字符串
					int length_y = hexString_y.length();
					char* buffer_y = new char[length_y / 2 + 1]; // 创建足够大小的字符数组来存放结果
					for (int i = 0; i < length_y; i += 2) {
					sscanf(hexString_y.substr(i, 2).c_str(), "%x", &buffer_y[i/2]); // 逐两位读取并转换成十六进制值
					}
					buffer_y[length_y / 2] = '\0'; // 在最后添加空字符作为字符串的结尾标志

				
					y=buffer_y;
					delete[] buffer_y; // 记得释放内存

					string theta = str.substr(64, 24);
					std::string hexString_theta = theta ; // 输入的16进制字符串
					int length_theta = hexString_theta.length();
					char* buffer_theta = new char[length_theta / 2 + 1]; // 创建足够大小的字符数组来存放结果
					for (int i = 0; i < length_theta; i += 2) {
					sscanf(hexString_theta.substr(i, 2).c_str(), "%x", &buffer_theta[i/2]); // 逐两位读取并转换成十六进制值
					}
					buffer_theta[length_theta / 2] = '\0'; // 在最后添加空字符作为字符串的结尾标志


					theta=buffer_theta;
					delete[] buffer_theta; // 记得释放内存

					float num_x=0,num_y=0,num_theta=0;
				
				
					if(x.find('-')<x.length()){
						x = x.substr(x.find('-'));
					}
					

					if(y.find('-')<y.length()){
						y = y.substr(y.find('-'));
					}
					if(theta.find('-')<theta.length()){
						theta = theta.substr(theta.find('-'));
					}
					 num_x = std::stof(x); // 调用 stof() 进行转换
					 num_y = std::stof(y); // 调用 stof() 进行转换
					 num_theta = std::stof(theta); // 调用 stof() 进行转换
					std::cout << "转换后的字符串为: " << num_x << std::endl;
					std::cout << "转换后的字符串为: " << num_y << std::endl;
					std::cout << "转换后的字符串为: " << num_theta << std::endl;
				
				
					//只通过yaw, 即绕z的旋转角度计算四元数，用于平面小车。返回四元数
					tf::createQuaternionMsgFromYaw(num_theta).w;
					p.pose.position.x =num_x;
					p.pose.position.y = num_y;
					p.pose.position.z = 0;
					p.pose.orientation.x = 	tf::createQuaternionMsgFromYaw(num_theta).x;
					p.pose.orientation.y = 	tf::createQuaternionMsgFromYaw(num_theta).y;
					p.pose.orientation.z = 	tf::createQuaternionMsgFromYaw(num_theta).z;
					p.pose.orientation.w =	tf::createQuaternionMsgFromYaw(num_theta).w;
					for (int i = 0; i < 20; i++)
					{
						point_pub.publish(p);
					}
				}
				if(sub.renwuleixing_r=="0c")
				{
					ros::param::set("renwu_finish",99);
					cout << "任务类型是源点复位" << endl;
					ros::param::set("task", 12);
					ros::param::set("work_state",2);
				}
				if(sub.renwuleixing_r=="0d")
				{
					cout << "任务类型是清除任务" << endl;
					ros::param::set("task", -1);//表示此时是执行的清除指令的发送0的cmd_vel
					ros::param::set("work_state",1);
					ros::param::set("renwu_finish",13);
				}
					
				
				send_msg. id_r="02";
				send_msg. gongneng_r="A2";
				send_msg. len_r="1B";
				send_msg. renwuliushui_r=liushui;
				send_msg. renwuzhuangtai_r="01";
				send_msg. over_r="FF";

				string send_msg2=send_msg. id_r
								+send_msg. gongneng_r
								+send_msg. len_r
								+send_msg. renwuliushui_r
								+send_msg. renwuzhuangtai_r
								+send_msg. over_r;

				const char *q;
				q = send_msg2.c_str();
				send(socket_cli, q, send_msg2.length(), 0);
				cout << "回复的内容为 " << send_msg2 << endl;
				ros::Rate rate(0.5);
				rate.sleep();
				//ros::param::set("renwu_finish",0);
				//ros::param::set("renwu_finish",99);
			}
			if(sub.gongneng=="a3"){//sub.gongneng=="A3"
				cout<<"---------------------------小车发送地图---------------------------"<<endl;
				map_updata();
			}
			if(sub.gongneng=="a4"){
				cout<<"---------------------------小车接受地图---------------------------"<<endl;
				cout<<str<<endl;
				send_all sub_map;
				sub_map.id_c="01";
				sub_map.gongneng_c = "A4";
				string send_msg1;
				send_msg1 = sub_map.id_c +
							sub_map.gongneng_c +
							"FFFFFFFF"+
							"01"+
							"FF";

				int size_by = send_msg1.length();
				char b[256];
				sprintf(b, "%d", size_by);


				const char *q;
				q = send_msg1.c_str();
				send(socket_cli, q, send_msg1.length(), 0);

			}
			if(sub.gongneng=="a6"){
			
				cout<<"---------------------------小车切换楼层---------------------------"<<endl;
				cout<<str<<endl;
				sub.floor=str.substr(6,2);
				//终端页面下达99的命令，此时是初始定位，在此处进行转换，发给脚本一楼
				if(sub.floor == "63"){
					ros::param::set("floor",1);
				}
				//当重新回到一楼时，终端页面下达位于一楼指令，在此处转换为99发给脚本，开启定位
				if(sub.floor == "01"){
					ros::param::set("floor",99);
				}
				if(sub.floor == "02"){
					ros::param::set("floor",2);
				}
				if(sub.floor == "03"){
					ros::param::set("floor",3);
				}
				if(sub.floor == "04"){
					ros::param::set("floor",4);
				}
				if(sub.floor == "05"){
					ros::param::set("floor",5);
				}
				if(sub.floor == "06"){
					ros::param::set("floor",6);
				}
				if(sub.floor == "07"){
					ros::param::set("floor",7);
				}
				if(sub.floor == "08"){
					ros::param::set("floor",8);
				}
				if(sub.floor == "0a"){
					ros::param::set("floor",10);
				}
				if(sub.floor == "0c"){
					ros::param::set("floor",12);
				}
				if(sub.floor == "0e"){
					ros::param::set("floor",14);
				}
				if(sub.floor == "10"){
					ros::param::set("floor",16);
				}
				if(sub.floor == "12"){
					ros::param::set("floor",18);
				}
				if(sub.floor == "14"){
					ros::param::set("floor",20);
				}
				if(sub.floor == "16"){
					ros::param::set("floor",22);
				}
				if(sub.floor == "18"){
					ros::param::set("floor",24);
				}
			
				
				
				send_all send_;
				send_.id_c="02";
				send_.gongneng_c = "A6";


				string send_msg1;
				send_msg1 = send_.id_c +
							send_.gongneng_c +
							"1B"+
							"01"+
							"FF";

				int size_by = send_msg1.length();
				char b[256];
				sprintf(b, "%d", size_by);
				const char *q;
				q = send_msg1.c_str();
				send(socket_cli, q, send_msg1.length(), 0);

			}
			if(sub.gongneng=="a7"){
			
				cout<<"---------------------------小车设置为急停与恢复---------------------------"<<endl;
				cout<<str<<endl;
				sub.jiting=str.substr(6,2);
				int jiting_begin = 0;
				int jiting_end = 0;
				if(sub.jiting == "01"){
					cout<<"输出为急停"<<endl;
					ros::param::get("task",jiting_begin);
					ros::param::set("task",-1);//task为-1才会进行急停的操作
					ros::param::set("jiting_begin",jiting_begin);
				}
				if(sub.jiting == "02"){
					cout<<"输出为急停恢复"<<endl;
					ros::param::get("jiting_begin",jiting_end);
					ros::param::set("task",jiting_end);
				}

				
				send_all send_;
				send_.id_c="02";
				send_.gongneng_c = "A7";


				string send_msg1;
				send_msg1 = send_.id_c +
							send_.gongneng_c +
							"1B"+
							"01"+
							"FF";

				int size_by = send_msg1.length();
				char b[256];
				sprintf(b, "%d", size_by);
				const char *q;
				q = send_msg1.c_str();
				send(socket_cli, q, send_msg1.length(), 0);

			}
			if(sub.gongneng=="a9"){
				cout<<"---------------------------小车设置为关闭所有终端与关机---------------------------"<<endl;
				cout<<str<<endl;
				ros::param::set("close_all",1);
				send_all send_;
				send_.id_c="02";
				send_.gongneng_c = "A9";
				string send_msg1;
				send_msg1 = send_.id_c +
							send_.gongneng_c +
							"01"+
							"01"+
							"FF";

				int size_by = send_msg1.length();
				char b[256];
				sprintf(b, "%d", size_by);
				const char *q;
				q = send_msg1.c_str();
				send(socket_cli, q, send_msg1.length(), 0);
			}
		task_.task_num=1;
		task_pub.publish(task_);

		map_get(recvbuf);
		rate1.sleep();
		}
		else{
			close(socket_cli);
			ros::Duration du(5);
			du.sleep();
			int socket_cli_3=socket(AF_INET,SOCK_STREAM,0);
			if( connect(socket_cli_3, (struct sockaddr*) &sev_addr, sizeof(sev_addr))< 0)
			{
				std::cout << "connect error 2" << std::endl;
				cout<<"socket_cli"<<endl;
				
			}
			else
			std::cout << "connected successfullly! 2" << std::endl;
		}
	}
	
	/*
	 *@fuc: 关闭连接
	 */
	close(socket_cli);
	return 0;
}
