#include "lidar_ros/lidar_core.h"

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "lidar_ros");
    ROS_WARN("lidar obstacle detection start!");

    ros::NodeHandle nh;
    
    LidarCore lidar_core(nh);
    return 0;
}
