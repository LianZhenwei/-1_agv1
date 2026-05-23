#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <geometry_msgs/Twist.h>
#include <iostream>
#include <ros/ros.h>
#include <vector>
#include <csignal>

using namespace std;

int sockfd_8000, newsockfd_8000;
struct sockaddr_in server_addr_8000, client_addr_8000;
socklen_t client_len_8000;

int sockfd_8001, newsockfd_8001;
struct sockaddr_in server_addr_8001, client_addr_8001;
socklen_t client_len_8001;

bool tcp_8000 = false;
bool tcp_8001 = false;


// 小车状态，需要实时更新
unsigned char state_data[14] = {0x01, 0xa1, 0x18, 0x02, 0x01, 0x01, 0x01, 0x01, 0x64, 0x01, 0x21, 0x01, 0x01, 0x01};

void sigintHandler(int sig)
{
  // 在节点输入Ctrl+C，退出节点时想要额外执行的操作

    //  关闭套接字
    close(newsockfd_8000);
    close(sockfd_8000);

    close(newsockfd_8001);
    close(sockfd_8001);

    // 调用ros关闭节点的函数
    ROS_INFO("shutting down!");
    ros::shutdown();
}

class TCP_Control_8000
{
public:
    TCP_Control_8000();
    ~TCP_Control_8000();

    void run();
    void stop();

private:
    unsigned char Receive_data_8000[4];

    void handle_data(unsigned char *Receive_data);
};

class TCP_Control_8001
{

public:

	TCP_Control_8001();
	~TCP_Control_8001();

	void run();
	void stop();

private:

    unsigned char Receive_data_8001[52];
	
	void handle_data(unsigned char *Receive_data_8001);

    void function_a2_action(unsigned char *Receive_data_8001);
    void function_a2_reply(unsigned char *Receive_data_8001);
    unsigned char function_a2_send_data[7] = {0x01,0xa2,0x06,0x01,0x21,0x01,0xff};

    void function_a6_action(unsigned char *Receive_data_8001);
    void function_a6_reply(unsigned char *Receive_data_8001);
    unsigned char function_a6_send_data[5] = {0x01,0xa6,0x01,0x01,0xff};

    void function_a7_action(unsigned char *Receive_data_8001);
    void function_a7_reply(unsigned char *Receive_data_8001);
    unsigned char function_a7_send_data[5] = {0x01,0xa7,0x01,0x01,0xff};

};

// 从给定的字符串中解析出一系列双精度浮点数
std::vector<double> extractDoublesFromCommaSeparated(const std::string& input) {
    std::vector<double> doubles;
    std::stringstream ss(input);
    std::string item;

    // 分割字符串，存储在字符串向量中
    std::vector<std::string> stringNumbers;
    while (std::getline(ss, item, ',')) {
        // 移动负号到前面
        auto pos = item.find('-');
        if (pos != std::string::npos) {
            item.erase(pos, 1);
            item.insert(0, "-");
        }
        stringNumbers.push_back(item);
    }
    //cout<<stringNumbers[0]<<""<<stringNumbers[1]<<""<<stringNumbers[2]<<endl;

    // 转换每个字符串为double，并存储在doubles向量中
    for (const std::string& str : stringNumbers) {
        try {
            doubles.push_back(std::stod(str));
        } catch (const std::invalid_argument& e) {
            std::cerr << "Invalid argument: " << str << std::endl;
        } catch (const std::out_of_range& e) {
            std::cerr << "Out of range: " << str << std::endl;
        }
    }

    return doubles;
}

//输入位姿hex，输出double位姿
void pose_analysis(unsigned char *Pose_data, std::vector<double> &pose_vector)
{
    //数组转字符串
    string str(reinterpret_cast<char*>(Pose_data), 38);
    cout << "str:" << str << endl;

    // 从字符串中解析出数字
    pose_vector = extractDoublesFromCommaSeparated(str);

    // std::cout << "Parsed Numbers: ";
    // for (double num : pose_vector)
    // {
    //     std::cout << num << " ";
    // }
    // std::cout << std::endl;
}



// 将16进制数组，转换成同值的十进制数组
int arrayToStr(unsigned char *buf, unsigned int buflen, char *out)
{
    char strBuf[33] = {0};
    char pbuf[32];
    int i;
    for(i = 0; i < buflen; i++)
    {
        sprintf(pbuf, "%02X", buf[i]);
        strncat(strBuf, pbuf, 2);
    }
    strncpy(out, strBuf, buflen * 2);
    //printf("out = %s\n", out);
    return buflen * 2;
}


//unsigned char 转 string
//string 转 unsigned char
void tcp_send_1(unsigned char *send_data, int length, char *out)
{
    //ROS_INFO("sizeof(send_data): %d",sizeof(send_data));
    // 将16进制数组，转换成同值的十进制数组
    char send_hex[2*length] = {0};
    arrayToStr(send_data, length, send_hex);


    // for(int i = 0; i<2*length;i++)
    // {
    //     ROS_INFO("%x",send_hex[i]);
    // }

    for (int i = 0; i < 2*length; i++)
    {
        out[i] = send_hex[i];
    }
    
}


// double数组转字符串
// 字符串转unsigned数组
void pose_send(std::vector<double> &pose, unsigned char *hex_pose)
{
    // 1、将double数组转成string
    std::string str_out;
    for (int i = 0; i < 3; i++)
    {
        double value = pose[i];
        std::stringstream ss;
        // 设置整数部分宽度为5，不足部分用0填充
        ss << std::setfill('0') << std::setw(5) << std::fixed << std::setprecision(0) << int(value);
        // 获取小数部分
        double decimalPart = value - int(value);
        // 设置小数部分宽度为6，不足部分用0填充
        ss << std::setfill('0') << std::setw(6) << std::setprecision(6) << decimalPart;
        std::string str = ss.str();
        //std::cout << "str_tem: " << str << std::endl;

        std::string tem1 = str;
        std::string tem2 = str;
        str.clear();

        int a = 0;
        if(value<0)
        {
            a = 1;
        }
        for (int i = 0; i < 5; i++)
        {
            str.push_back(tem1[i]);
        }
        for (int i = 6+a; i < 13+a; i++)
        {
            str.push_back(tem2[i]);
        }

        //std::cout << "str: " << str << std::endl;
        str_out += str;

        if (i < 2)
        {
            std::string ttem = ",";
            str_out.push_back(ttem[0]);
        }
    }
    //std::cout << "str_out: " << str_out << std::endl;

    // 2、string转unsigned char数组

    for (size_t i = 0; i < str_out.length(); ++i) 
    {
        // 将字符转换为十六进制数
        unsigned char hex = str_out[i];

        hex_pose[i] = hex;
    }

    // for(int i = 0; i<38;i++)
    // {
    //     ROS_INFO("%x",hex_pose[i]);
    // }

}
