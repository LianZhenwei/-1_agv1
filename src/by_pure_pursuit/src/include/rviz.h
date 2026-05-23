#ifndef _RVIZ_H_
#define _RVIZ_H_
#include "headfile.h"
string my_frame_id = "odom";

void rviz_track_point(ros::Publisher marker_pub, Point point)
{
    visualization_msgs::Marker points;
    points.header.frame_id = my_frame_id;
    points.header.stamp = ros::Time::now();
    points.ns = "points_and_lines";
    points.action = visualization_msgs::Marker::ADD;
    points.pose.orientation.w = 1.0;
    points.id = 0;
    points.type = visualization_msgs::Marker::POINTS;
    points.scale.x = 0.5;
    points.scale.y = 0.5;
    points.color.g = 1.0f;
    points.color.a = 1.0;
    points.color.b = 1.0;

    geometry_msgs::Point p;
    p.x = point.x;
    p.y = point.y;
    p.z = 0;
    points.points.push_back(p);

    marker_pub.publish(points);
}
void rviz_road(ros::Publisher marker_pub, std::vector<Point> roads, double r, double g, double b)
{
    // lzw_1113：
    if (roads.empty())
    {
        ROS_WARN("路径为空，不发布可视化Marker");
        return;
    }
    if (roads.size() < 2) // 对于LINE_STRIP类型，至少需要2个点
    {
        ROS_WARN("路径点数不足2个，不发布LINE_STRIP类型的Marker");
        return;
    }
    for (const auto &point : roads) // 检查点坐标是否有效
    {
        if (std::isnan(point.x) || std::isnan(point.y) ||
            std::isinf(point.x) || std::isinf(point.y))
        {
            ROS_WARN("发现无效坐标点，不发布可视化Marker");
            return;
        }
    }
    visualization_msgs::Marker points;
    points.header.frame_id = my_frame_id;
    points.header.stamp = ros::Time::now();
    points.ns = "points_and_lines";
    points.action = visualization_msgs::Marker::ADD;
    points.pose.orientation.w = 1.0;
    points.id = 0;
    points.type = visualization_msgs::Marker::POINTS;
    points.scale.x = 0.4;
    points.scale.y = 0.4;
    points.color.r = r;
    points.color.g = g;
    points.color.b = b;
    points.color.a = 1.0;

    for (size_t j = 0; j < roads.size(); j++)
    {
        geometry_msgs::Point p;
        p.x = roads[j].x;
        p.y = roads[j].y;
        p.z = 0;
        points.points.push_back(p);
    }
    marker_pub.publish(points);
}

#endif