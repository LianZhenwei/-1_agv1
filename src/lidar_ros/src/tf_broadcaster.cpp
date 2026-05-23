#include <ros/ros.h> 
#include <tf/transform_broadcaster.h> 
int main(int argc, char** argv) 
{ 
    ros::init(argc, argv, "tf_broadcaster_node"); 
    ros::NodeHandle n; 
    ros::Rate loop_rate(100); 
    tf::TransformBroadcaster broadcaster; 
    tf::Transform base_laser2base_link; // 以 base_laser 为父坐标系， base_link为子坐标系
    base_laser2base_link.setOrigin(tf::Vector3(-0.1, 0.0, -0.2)); // 子坐标系在父坐标系中的位置
    base_laser2base_link.setRotation(tf::Quaternion(0, 0, 0, 1)); 
    while(n.ok()) 
    { 
        // 发布 坐标变换信息 到话题 /tf
        // 注意坐标系变换 和 坐标变换
        // 函数发布一个从 父坐标系 到 子坐标系 的变换关系。以 父坐标系base_laser 为基准
        // 从 parent到 child的坐标系变换（frame transform）等同于把一个点从 child坐标系向 parent坐标系的坐标变换，等于 child坐标系在 parent坐标系的姿态描述
        // broadcaster.sendTransform(tf::StampedTransform(base_laser2base_link,ros::Time::now(), "base_laser", "base_link")); 
        broadcaster.sendTransform(tf::StampedTransform(base_laser2base_link,ros::Time::now(), "body", "base_link_centre")); 
        // broadcaster.sendTransform(tf::StampedTransform(tf::Transform(tf::Quaternion(0, 0, 0, 0), tf::Vector3(1, 0, 0)),ros::Time::now(),"base_link", "base_laser")); 
        loop_rate.sleep(); 
    } 
    return 0; 
}

