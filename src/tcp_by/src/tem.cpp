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
#include <fcntl.h>
#include <sys/shm.h>
#include "geometry_msgs/PoseStamped.h"
#include <fstream>
#include "math.h"
#include "tcp_by/task.h"
#include "tf/tf.h"
#include <csignal>

using namespace std;
int tilong_;
int renwu_finish = 0;
int liushui_int;
string liushui_globle = "0000";
vector<string> map_msg;
#define    BUF_SIZE   100 

struct sub_all
{
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
struct send_all
{
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

    string id_r;
    string gongneng_r;
    string len_r;
    string renwuliushui_r;
    string renwuzhuangtai_r;
    string over_r;

    string road_id;
    vector<pair<string, string>> point;
    vector<string> beh;
    vector<string> pre;
};

char *SERVER_IP = "192.168.3.3";
int sockfd_8000, newsockfd_8000;
struct sockaddr_in server_addr_8000, client_addr_8000;
socklen_t client_len_8000;

void sigintHandler(int sig)
{
  // 在节点输入Ctrl+C，退出节点时想要额外执行的操作

    //  关闭套接字
    close(newsockfd_8000);
    close(sockfd_8000);


    // 调用ros关闭节点的函数
    ROS_INFO("shutting down!");
    ros::shutdown();
}



void string2hexString(char *input, char *output)
{
    int loop;
    int i;

    i = 0;
    loop = 0;

    while (input[loop] != '\0')
    {
        sprintf((char *)(output + i), "%02X", input[loop]);
        loop += 1;
        i += 2;
        // cout<<i<<endl;
    }
    // insert NULL at the end of the output string
    output[i++] = '\0';
}
std::string HexToString(const char *hex, size_t size)
{

    std::stringstream ss;

    for (size_t i = 0; i < size; ++i)
    {

        ss << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(hex[i]);
    }

    return ss.str();
}
std::string floatToString(float value)
{

    std::ostringstream stream;

    stream << std::fixed << std::setprecision(2) << value;

    return stream.str();
}
std::string hexToString1(const std::string &hex)
{
    std::stringstream ss;
    ss << std::hex;
    for (char c : hex)
    {
        ss >> c;
    }
    std::string result;
    string c;
    while (ss >> c)
    {
        result += c;
    }
    return result;
}
std::string padZero(std::string str, int targetLength)
{

    while (str.length() < targetLength)
    {
        str = "0" + str;
    }

    return str;
}



void init()
{
    // 创建套接字
    sockfd_8000 = socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd_8000 < 0)
    {
        ROS_ERROR("Error while opening socket: %s", strerror(errno));
        return;
    }

    // 设置服务器地址结构
    memset(&server_addr_8000, 0, sizeof(server_addr_8000));
    server_addr_8000.sin_family = AF_INET;
    server_addr_8000.sin_port = htons(8000);       // 端口号
    server_addr_8000.sin_addr.s_addr = 0 ;//inet_addr(SERVER_IP);; // 接受固定 IP 地址："192.168.3.3"

    // 绑定套接字
    while (bind(sockfd_8000, (struct sockaddr *)&server_addr_8000, sizeof(server_addr_8000)) < 0)
    {
        ROS_ERROR("Error on binding: %s", strerror(errno));
        //close(sockfd_8000);
        //return;
    }

    // 设置套接字超时(5秒)
    struct timeval timeout;
    timeout.tv_sec = 2;  // 超时时间秒数
    timeout.tv_usec = 0; // 微秒
    setsockopt(sockfd_8000, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));

    // 监听连接请求
    listen(sockfd_8000, 5); // 最大挂起连接数为 5
    ROS_INFO("Server is listening on port 8000...");

    client_len_8000 = sizeof(client_addr_8000);

    // 接受客户端连接
    while (newsockfd_8000==-1)
    {
   newsockfd_8000 = accept(sockfd_8000, (struct sockaddr *)&client_addr_8000, &client_len_8000);
   ROS_INFO("connecting");
    }
    
    
    // if (newsockfd_8000 < 0)
    // {
    //     ROS_ERROR("Error on accept: %s", strerror(errno));
    //     close(sockfd_8000);
    //     return;
    // }

    ROS_INFO("Client_8000 connected!");
}

int main(int argc, char **argv)
{
    // 覆盖原来的Ctrl+C中断函数，原来的只会调用ros::shutdown(）
    signal(SIGINT, sigintHandler);

    ros::init(argc, argv, "tcp_client_node");
    ros::NodeHandle n;
    tcp_by::task task_;
    sub_all sub;
    send_all send_msg;

    init();

    while (ros::ok)
    {
        char recvbuf[BUF_SIZE];

        cout << "-------TCP_8000_Listening--------" << endl;

        struct timeval timeout;
        timeout.tv_sec = 2;  // 超时时间秒数
        timeout.tv_usec = 0; // 微秒
        setsockopt(sockfd_8000, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
        // 接收
        int n = read(newsockfd_8000, recvbuf, 100);
        cout<<"-------------------read"<<endl;
        if (n < 0)
        {
            // ROS_ERROR("Error reading from socket: %s", strerror(errno));
            ROS_ERROR("tcp_8000 receive error or no msg send");
        }
        else
        {
            ROS_INFO("Receive_data: %x", recvbuf[0]);
            ROS_INFO("Receive_data: %x", recvbuf[1]);
            ROS_INFO("Receive_data: %x", recvbuf[2]);
            ROS_INFO("Receive_data: %x", recvbuf[3]);
        }

        // declare output string with double size of input string
        // because each character of input string will be converted
        // in 2 bytes
        ros::Rate rate1(1);
        string str = "00";
        str = HexToString(recvbuf, sizeof(recvbuf));
        cout << str.length() << endl;
        cout << "接收到的数据为  " << str << endl;
        sub.gongneng = str.substr(2, 2);
        if (sub.gongneng == "a1")
        {
            cout << "---------------------------小车状态查询---------------------------" << endl;
            send_msg.id_c = "02";       // 小车id
            send_msg.gongneng_c = "A1"; // 功能
            send_msg.len_c = "41";      // 数据长度
            // 控制权限(设备状态)
            int kongzhiquanxian = 0;
            ros::param::get("kongzhiquanxian", kongzhiquanxian);
            if (kongzhiquanxian == 1)
            {
                send_msg.state_shebei_c = "02";
            }
            else
            {
                send_msg.state_shebei_c = "01";
            }
            // 工作状态
            int work_state = 0;
            ros::param::get("work_state", work_state);
            if (work_state == 1)
            {
                send_msg.state_gongzuo_c = "01";
                cout << "空闲" << endl;
            }
            else if (work_state == 2)
            {
                cout << "忙碌" << endl;
                send_msg.state_gongzuo_c = "02";
            }
            else
            {
                send_msg.state_gongzuo_c = "01";
            }
            // 升降状态
            send_msg.shengjiang_c = "02";
            // 载货状态
            send_msg.zaihuo_c = "01";
            // 任务完成状态
            ros::param::get("renwu_finish", renwu_finish);
            if (renwu_finish == 0)
            {
                send_msg.renwu_finish_c = "00";
            }
            else if (renwu_finish == 1)
            {
                send_msg.renwu_finish_c = "01";
                // 已移动至目的经纬度
            }
            else if (renwu_finish == 2)
            {
                send_msg.renwu_finish_c = "02";
            }
            else if (renwu_finish == 3)
            {
                send_msg.renwu_finish_c = "03";
            }
            else if (renwu_finish == 4)
            {
                send_msg.renwu_finish_c = "04";
            }
            else if (renwu_finish == 5)
            {
                send_msg.renwu_finish_c = "05";
            }
            else if (renwu_finish == 6)
            {
                send_msg.renwu_finish_c = "06";
                // 已到梯笼前定位成功
            }
            else if (renwu_finish == 7)
            {
                send_msg.renwu_finish_c = "07";
                // 已进梯笼
            }
            else if (renwu_finish == 8)
            {
                send_msg.renwu_finish_c = "08";
                // 已出梯笼
            }
            else if (renwu_finish == 9)
            {
                send_msg.renwu_finish_c = "09";
            }
            else if (renwu_finish == 10)
            {
                send_msg.renwu_finish_c = "10";
            }
            // 当前电量
            int dianliang_send = 12;
            // ros::param::get("dianliang",dianliang_send);
            string dianchi_str = std::to_string(dianliang_send);
            cout << "dianchi_str" << dianchi_str << endl;
            send_msg.dianliang_c = dianchi_str;
            // 任务流水
            string liushui = "0000";
            ros::param::get("liushui", liushui);
            send_msg.renwuliushui_c = liushui;
            // 故障代码
            send_msg.guzhang_c = "00";
            // 场景id
            send_msg.changjingid = "01";
            // 当前楼层
            int floor = 1;
            string floor_str = "01";
            ros::param::get("floor", floor);
            if (floor == 1)
            {
                floor_str = "01";
            }
            if (floor == 2)
            {
                floor_str = "02";
            }
            if (floor == 3)
            {
                floor_str = "03";
            }
            if (floor == 4)
            {
                floor_str = "01";
            }
            send_msg.louceng = floor_str;
            // 位置
            float x_ = 0, y_ = 0, theta_ = 0;
            ros::param::get("x", x_);
            ros::param::get("y", y_);
            ros::param::get("theta", theta_);
            string x = floatToString(x_);
            string y = floatToString(y_);
            string theta = floatToString(theta_);
            x = padZero(x, 12);
            y = padZero(y, 12);
            theta = padZero(theta, 12);
            send_msg.weizhi_c = x + "," + y + "," + theta;
            // 完成位
            send_msg.over_c = "FF";
            string send_msg1;
            send_msg1.append(send_msg.id_c);
            send_msg1.append(send_msg.gongneng_c);
            send_msg1.append(send_msg.len_c);
            send_msg1.append(send_msg.state_shebei_c);
            send_msg1.append(send_msg.state_gongzuo_c);
            send_msg1.append(send_msg.shengjiang_c);
            send_msg1.append(send_msg.zaihuo_c);
            send_msg1.append(send_msg.renwu_finish_c);
            send_msg1.append(send_msg.dianliang_c);
            send_msg1.append(send_msg.renwuliushui_c);
            send_msg1.append(send_msg.guzhang_c);
            send_msg1.append(send_msg.changjingid);
            send_msg1.append(send_msg.louceng);
            send_msg1.append(send_msg.weizhi_c);
            send_msg1.append(send_msg.over_c);

            cout << "回复的内容为 " << send_msg1 << endl;
            const char *q;
            q = send_msg1.c_str();

            // 发送至客户端
            int n = write(newsockfd_8000, q, send_msg1.length());
            if (n < 0)
            {
                ROS_ERROR("Error writing to socket: %s", strerror(errno));
            }

            ros::Duration(0.02).sleep();
        }
        rate1.sleep();
        ros::spinOnce();
    }

    // 关闭套接字
    close(newsockfd_8000);
    close(sockfd_8000);

    return 0;
}