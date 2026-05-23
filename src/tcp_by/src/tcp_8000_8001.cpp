#include <tcp_8000_8001.h>

int main(int argc, char **argv)
{
    //设置中文编码
    //setlocale(LC_ALL,"");

    ros::init(argc, argv, "All_Control_node");
    ros::NodeHandle nh_;

    // 覆盖原来的Ctrl+C中断函数，原来的只会调用ros::shutdown(）
    signal(SIGINT, sigintHandler);

    TCP_Control_8000 tcp_control_8000;
    TCP_Control_8001 tcp_control_8001;

    ros::Rate loop_rate(10);
    while (ros::ok())
    {
        if(tcp_8000)
        {
            tcp_control_8000.run();
        }

        if(tcp_8001)
        {
            tcp_control_8001.run();
        }

        loop_rate.sleep();
    }

    return 0;
}

TCP_Control_8000::TCP_Control_8000()
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
    server_addr_8000.sin_addr.s_addr = INADDR_ANY; // 接受任意 IP 地址

    // 绑定套接字
    if (bind(sockfd_8000, (struct sockaddr *)&server_addr_8000, sizeof(server_addr_8000)) < 0)
    {
        ROS_ERROR("Error on binding: %s", strerror(errno));
        close(sockfd_8000);
        return;
    }

    // 设置套接字超时(2秒)
    struct timeval timeout;
    timeout.tv_sec = 2;  // 超时时间秒数
    timeout.tv_usec = 0; // 微秒
    setsockopt(sockfd_8000, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));

    // 监听连接请求
    listen(sockfd_8000, 5); // 最大挂起连接数为 5
    ROS_INFO("Server is listening on port 8000...");

    client_len_8000 = sizeof(client_addr_8000);

    // 接受客户端连接
    newsockfd_8000 = accept(sockfd_8000, (struct sockaddr *)&client_addr_8000, &client_len_8000);
    if (newsockfd_8000 < 0)
    {
        ROS_ERROR("Error on accept: %s", strerror(errno));
        close(sockfd_8000);
        return;
    }

    //初始化tcp_8000成功
    tcp_8000 = true;
    ROS_INFO("Client_8000 connected!");

}

TCP_Control_8000::~TCP_Control_8000()
{
    // 关闭套接字
    close(newsockfd_8000);
    close(sockfd_8000);
}

void TCP_Control_8000::run()
{
    cout << "-------TCP_8000_Listening--------" << endl;

    // 接收
    int n = read(newsockfd_8000, Receive_data_8000, 4);
    if (n < 0)
    {
        //ROS_ERROR("Error reading from socket: %s", strerror(errno));
        ROS_ERROR("tcp_8000 receive error or no msg send");
    }
    else
    {
        ROS_INFO("Receive_data: %x", Receive_data_8000[0]);
        ROS_INFO("Receive_data: %x", Receive_data_8000[1]);
        ROS_INFO("Receive_data: %x", Receive_data_8000[2]);
        ROS_INFO("Receive_data: %x", Receive_data_8000[3]);
        // 回复信息
        // 需要加入小车ID判断
        handle_data(Receive_data_8000);
    }
}

void TCP_Control_8000::handle_data(unsigned char *Receive_data_8000)
{
    //1、将状态数组转成字符串
    char state_[28];
    tcp_send_1(state_data, 14, state_);

    //2、订阅位姿信息，转成char数组
    std::vector<double> pose = {-4.12, 0.06, 3.14};
    unsigned char hex_pose[38];
    pose_send(pose, hex_pose);

    //3、将两个数组合并，并加上结尾ff
    unsigned char send_data[68] = {};
    for (int i = 0; i < 28; i++)
    {
        send_data[i] = state_[i];
    }
    for (int i = 0; i < 38; i++)
    {
        send_data[i + 28] = hex_pose[i];
    }
    unsigned char end[2] = {0x46, 0x46};
    send_data[66] = end[0];
    send_data[67] = end[1];

    // 发送至客户端
    int n = write(newsockfd_8000, send_data, 68);
    if (n < 0)
    {
        ROS_ERROR("Error writing to socket: %s", strerror(errno));
    }

    ros::Duration(0.02).sleep();
}

TCP_Control_8001::TCP_Control_8001()
{
    // 创建套接字
    sockfd_8001 = socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd_8001 < 0)
    {
        ROS_ERROR("Error while opening socket: %s", strerror(errno));
        return;
    }

    // 设置服务器地址结构
    memset(&server_addr_8001, 0, sizeof(server_addr_8001));
    server_addr_8001.sin_family = AF_INET;
    server_addr_8001.sin_port = htons(8001);       // 端口号
    server_addr_8001.sin_addr.s_addr = INADDR_ANY; // 接受任意 IP 地址

    // 绑定套接字
    if (bind(sockfd_8001, (struct sockaddr *)&server_addr_8001, sizeof(server_addr_8001)) < 0)
    {
        ROS_ERROR("Error on binding: %s", strerror(errno));
        close(sockfd_8001);
        return;
    }

    // 设置套接字超时(5秒)
    struct timeval timeout;
    timeout.tv_sec = 2;  // 超时时间秒数
    timeout.tv_usec = 0; // 微秒
    setsockopt(sockfd_8001, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));

    // 监听连接请求
    listen(sockfd_8001, 5); // 最大挂起连接数为 5
    ROS_INFO("Server is listening on port 8001...");

    client_len_8001 = sizeof(client_addr_8001);

    // 接受客户端连接
    newsockfd_8001 = accept(sockfd_8001, (struct sockaddr *)&client_addr_8001, &client_len_8001);
    if (newsockfd_8001 < 0)
    {
        ROS_ERROR("Error on accept: %s", strerror(errno));
        close(sockfd_8001);
        return;
    }

    //初始化tcp_8001成功
    tcp_8001 = true;
    ROS_INFO("Client_8001 connected!");

}

TCP_Control_8001::~TCP_Control_8001()
{
    // 关闭套接字
    close(newsockfd_8001);
    close(sockfd_8001);
}

void TCP_Control_8001::run()
{
    cout << "-------TCP_8001_Listening--------" << endl;

        // 接收
        int n = read(newsockfd_8001, Receive_data_8001, 68);
        if (n < 0)
        {
            //ROS_ERROR("Error reading from socket: %s", strerror(errno));
            ROS_ERROR("tcp_8001 receive error or no msg send");
        }
        else
        {
            ROS_INFO("Receive_data: %x", Receive_data_8001[0]);
            ROS_INFO("Receive_data: %x", Receive_data_8001[1]);
            ROS_INFO("Receive_data: %x", Receive_data_8001[2]);
            ROS_INFO("Receive_data: %x", Receive_data_8001[3]);
            // 回复信息
            handle_data(Receive_data_8001);
        }

}

void TCP_Control_8001::handle_data(unsigned char *Receive_data_8001)
{
    //调度指令
    if(Receive_data_8001[1] == 0xa2)
    {
        ROS_INFO("受到系统调度指令");

        //回复UMTS
        function_a2_reply(Receive_data_8001);

        //执行指令
        function_a2_action(Receive_data_8001);

    }
    //发送小车地图到UMTS
    else if(Receive_data_8001[1] == 0xa3)
    {

    }
    //UMTS发送地图到小车
    else if(Receive_data_8001[1] == 0xa4)
    {

    }
    //UMTS同步地图到小车
    else if(Receive_data_8001[1] == 0xa5)
    {

    }
    //修改小车楼层数值
    else if(Receive_data_8001[1] == 0xa6)
    {
        ROS_INFO("修改小车楼层");

        //回复UMTS
        function_a6_reply(Receive_data_8001);

        //执行指令
        function_a6_action(Receive_data_8001);
    }
    //急停
    else if(Receive_data_8001[1] == 0xa7)
    {
        ROS_WARN("急停");

        //回复UMTS
        function_a7_reply(Receive_data_8001);

        //执行指令
        function_a7_action(Receive_data_8001);
    }
    else
    {
        ROS_WARN("Error about received data!!");
    }

}

void TCP_Control_8001::function_a2_reply(unsigned char *Receive_data_8001)
{
    //更新任务流水号
    function_a2_send_data[3] = Receive_data_8001[3];
    function_a2_send_data[4] = Receive_data_8001[4];

    char send_data[14];
    tcp_send_1(function_a2_send_data,7,send_data);

    // 发送回复
    int n = write(newsockfd_8001, send_data, 14);
    if (n < 0)
    {
        ROS_ERROR("Error writing to socket: %s", strerror(errno));
    }

    ros::Duration(0.02).sleep();
}

void TCP_Control_8001::function_a2_action(unsigned char *Receive_data_8001)
{
    //移动到指定位置
    if(Receive_data_8001[5] == 0x01)
    {

    }
    //顶升
    else if(Receive_data_8001[5] == 0x02)
    {

    }
    //下降
    else if(Receive_data_8001[5] == 0x03)
    {

    }
    //充电
    else if(Receive_data_8001[5] == 0x04)
    {

    }
    //停止充电
    else if(Receive_data_8001[5] == 0x05)
    {

    }
    //梯笼前定位
    else if(Receive_data_8001[5] == 0x06)
    {

    }
    //进梯笼
    else if(Receive_data_8001[5] == 0x07)
    {

    }
    //出梯笼
    else if(Receive_data_8001[5] == 0x08)
    {

    }
    //进垛位
    else if(Receive_data_8001[5] == 0x09)
    {

    }
    //出垛位
    else if(Receive_data_8001[5] == 0x10)
    {

    }
    else
    {
        ROS_WARN("a2,调度指令，任务错误！");
    }
}

void TCP_Control_8001::function_a6_reply(unsigned char *Receive_data_8001)
{
    char send_data[10];
    tcp_send_1(function_a6_send_data,5,send_data);

    // 发送回复
    int n = write(newsockfd_8001, send_data, 10);
    if (n < 0)
    {
        ROS_ERROR("Error writing to socket: %s", strerror(errno));
    }

    ros::Duration(0.02).sleep();
}

void TCP_Control_8001::function_a6_action(unsigned char *Receive_data_8001)
{

}

void TCP_Control_8001::function_a7_reply(unsigned char *Receive_data_8001)
{
    char send_data[10];
    tcp_send_1(function_a7_send_data,5,send_data);


    // 发送回复
    int n = write(newsockfd_8001, send_data, 10);
    if (n < 0)
    {
        ROS_ERROR("Error writing to socket: %s", strerror(errno));
    }

    ros::Duration(0.02).sleep();
}

void TCP_Control_8001::function_a7_action(unsigned char *Receive_data_8001)
{

}

