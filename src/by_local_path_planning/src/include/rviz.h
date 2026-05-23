#ifndef _RVIZ_H_
#define _RVIZ_H_
#include "headfile.h"
string my_frame_id ="odom";
void erweima_rviz(ros::Publisher marker_pub, std::vector<Box_2d> vel_obs_info){
        visualization_msgs::Marker line_list;
        line_list.header.frame_id = my_frame_id;
        line_list.header.stamp = ros::Time::now();
        line_list.ns = "points_and_lines";
        line_list.action = visualization_msgs::Marker::ADD;
        line_list.pose.orientation.w = 1.0;
        line_list.id = 2;
        line_list.type = visualization_msgs::Marker::LINE_LIST;
        line_list.scale.x = 0.2;
        // Line list is red
        line_list.color.g = 1.0;
        line_list.color.a = 1.0;
        // Create the vertices for the points
        geometry_msgs::Point p;
        for (uint32_t i = 0; i < vel_obs_info.size(); i++)
        {
            for (uint32_t j = 0; j < vel_obs_info[i].Box2d_corner.size(); j++)
            {
                double y = vel_obs_info[i].Box2d_corner[j].y;
                double x = vel_obs_info[i].Box2d_corner[j].x;
                float z = 0;
                geometry_msgs::Point p;
                p.x = x;
                p.y = y;
                p.z = z;
                line_list.points.push_back(p);
            }
        }
        marker_pub.publish(line_list);
}

#endif