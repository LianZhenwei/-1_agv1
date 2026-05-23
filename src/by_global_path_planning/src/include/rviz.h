#ifndef _RVIZ_H_
#define _RVIZ_H_
#include "headfile.h"
string my_frame_id ="odom";

void  rviz_road(ros::Publisher marker_pub, std::vector<Road> roads){
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
    points.color.g = 1.0f;
    points.color.a = 1.0;

    for (size_t i = 0; i < roads.size(); i++)
    {
        for (size_t j = 0; j < roads[i].road_points.size(); j++)
        {
            geometry_msgs::Point p;
            p.x = roads[i].road_points[j].x;
            p.y = roads[i].road_points[j].y;
            p.z = 0;
            points.points.push_back(p);
        }
    }
    marker_pub.publish(points);
}

void rviz_veh_box2d(ros::Publisher marker_pub, Box_2d veh_info){
   visualization_msgs::Marker points;
    points.header.frame_id =  my_frame_id;
    points.header.stamp = ros::Time::now();
    points.ns = "points_and_lines";
    points.action =  visualization_msgs::Marker::ADD;
    points.pose.orientation.w =  1.0;
    points.id = 0;
    points.type = visualization_msgs::Marker::POINTS;
    // POINTS markers use x and y scale for width/height respectively
    points.scale.x = 5;
    points.scale.y = 5;
    // Points are green
    points.color.g = 1.0f;
    points.color.a = 1.0;
    // Create the vertices for the points   
       //draw vehicle
     visualization_msgs::Marker line_list1;
     line_list1.header.frame_id = "odom";
    line_list1.header.stamp = ros::Time::now();
     line_list1.ns = "points_and_lines";
    line_list1.action = visualization_msgs::Marker::ADD;
    line_list1.pose.orientation.w = 1.0;
    line_list1.id = 2;
    line_list1.type = visualization_msgs::Marker::LINE_LIST;
    line_list1.scale.x = 0.1;
    // Line list is blue
    line_list1.color.b = 1.0;
    line_list1.color.a = 1.0;
         for (uint32_t j= 0; j<veh_info.Box2d_corner.size() ;j++ )
            {    double y = veh_info.Box2d_corner[j].y;
                double  x =veh_info.Box2d_corner[j].x;
                float z = 0;
                geometry_msgs::Point p;
                      p.x = x;
                      p.y = y;
                     p.z = z;
                    line_list1.points.push_back(p);
         }    
    marker_pub.publish(line_list1);
}

void rviz_global_path_planning(ros::Publisher marker_pub, by_global_path_planning::Path globle_path){
    visualization_msgs::Marker points;
    points.header.frame_id = my_frame_id;
    points.header.stamp = ros::Time::now();
    points.ns = "points_and_lines";
    points.action =  visualization_msgs::Marker::ADD;
    points.pose.orientation.w =  1.0;
    points.id = 0;
    points.type = visualization_msgs::Marker::LINE_STRIP;
    points.scale.x = 0.05;
    points.scale.y = 0.05;
    points.color.r = 1.0f;
    points.color.a = 1.0;
    for (size_t i = 0; i < globle_path.points.size(); i++)
    {  
             geometry_msgs::Point p;
             p.x =  globle_path.points[i].x ;
             p.y =  globle_path.points[i].y;
             p.z = 0;
             points.points.push_back(p);        
    }
    marker_pub.publish(points);
}

#endif