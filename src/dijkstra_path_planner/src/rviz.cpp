#include "dijkstra_path_planner/rviz.h"

RvizVisualizer::RvizVisualizer(ros::NodeHandle &nh)
{
    pub_roads_ = nh.advertise<visualization_msgs::MarkerArray>("new_road_rviz", 10);
    pub_vertices_ = nh.advertise<visualization_msgs::MarkerArray>("new_vertex_rviz", 10);
    pub_global_path_ = nh.advertise<visualization_msgs::Marker>("new_global_path_rviz", 10);
    pub_vehicle_ = nh.advertise<visualization_msgs::Marker>("new_veh_rviz", 10);
    pub_goal_ = nh.advertise<visualization_msgs::Marker>("new_goal_rviz", 10);
}
std::vector<geometry_msgs::Point> RvizVisualizer::calculateVehicleRectVertices()
{
    std::vector<geometry_msgs::Point> local_vertices;
    geometry_msgs::Point p;
    p.x = vehicle_length_ / 2.0;
    p.y = vehicle_width_ / 2.0;
    p.z = 0.0;
    local_vertices.push_back(p);
    p.x = vehicle_length_ / 2.0;
    p.y = -vehicle_width_ / 2.0;
    p.z = 0.0;
    local_vertices.push_back(p);
    p.x = -vehicle_length_ / 2.0;
    p.y = -vehicle_width_ / 2.0;
    p.z = 0.0;
    local_vertices.push_back(p);
    p.x = -vehicle_length_ / 2.0;
    p.y = vehicle_width_ / 2.0;
    p.z = 0.0;
    local_vertices.push_back(p);
    return local_vertices;
}
geometry_msgs::Point RvizVisualizer::rotatePoint(const geometry_msgs::Point &point, double yaw)
{
    geometry_msgs::Point rotated_p;
    rotated_p.x = point.x * cos(yaw) - point.y * sin(yaw);
    rotated_p.y = point.x * sin(yaw) + point.y * cos(yaw);
    rotated_p.z = point.z; 
    return rotated_p;
}

void RvizVisualizer::publishRoads(const std::map<std::string, std::vector<geometry_msgs::Point>> &road_points)
{
    visualization_msgs::MarkerArray marker_array;
    resetMarkerId();
    for (const auto &road : road_points)
    {
        if (road.second.size() < 2)
        {
            continue;
        }
        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time::now();
        marker.id = marker_id_++;
        marker.type = visualization_msgs::Marker::LINE_STRIP; 
        marker.action = visualization_msgs::Marker::ADD;
        marker.scale.x = 0.1;                               
        marker.color.r = 0.0;                               
        marker.color.g = (marker_id_ % 2 == 0) ? 1.0 : 0.0; 
        marker.color.b = (marker_id_ % 2 == 1) ? 1.0 : 0.0; 
        marker.color.a = 0.8; 
        for (const auto &point : road.second)
        {
            marker.points.push_back(point);
        }
        marker_array.markers.push_back(marker);
    }
    pub_roads_.publish(marker_array);
}

void RvizVisualizer::publishVertices(const std::map<int, geometry_msgs::Point> &node_coords)
{
    visualization_msgs::MarkerArray marker_array;
    resetMarkerId();
    for (const auto &node : node_coords)
    {
        int node_id = node.first;
        const geometry_msgs::Point &node_pos = node.second;
        visualization_msgs::Marker sphere_marker;
        sphere_marker.header.frame_id = "map";
        sphere_marker.header.stamp = ros::Time::now();
        sphere_marker.id = marker_id_++;                        
        sphere_marker.type = visualization_msgs::Marker::SPHERE;
        sphere_marker.action = visualization_msgs::Marker::ADD;
        sphere_marker.scale.x = 0.3; 
        sphere_marker.scale.y = 0.3;
        sphere_marker.scale.z = 0.3;
        sphere_marker.color.r = 0.0;
        sphere_marker.color.g = 0.0;
        sphere_marker.color.b = 1.0;
        sphere_marker.color.a = 1.0;
        sphere_marker.pose.position = node_pos; 
        marker_array.markers.push_back(sphere_marker);

        visualization_msgs::Marker text_marker;
        text_marker.header.frame_id = "map";
        text_marker.header.stamp = ros::Time::now();
        text_marker.id = marker_id_++;                                   
        text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        text_marker.action = visualization_msgs::Marker::ADD;
        text_marker.pose.position.x = node_pos.x + 0.3;
        text_marker.pose.position.y = node_pos.y;      
        text_marker.pose.position.z = node_pos.z + 0.1;
        text_marker.scale.z = 1.3;
        text_marker.color.r = 0.0; 
        text_marker.color.g = 0.0;
        text_marker.color.b = 0.0;
        text_marker.color.a = 1.0; 
        text_marker.text = "节点" + std::to_string(node_id);
        marker_array.markers.push_back(text_marker);
    }
    pub_vertices_.publish(marker_array);
}

void RvizVisualizer::publishGlobalPath(const nav_msgs::Path &global_path)
{
    if (global_path.poses.size() < 2) 
    {
        ROS_WARN("Skipping publish: Global path has fewer than 2 points.");
        return;
    }
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();
    marker.id = 0;
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = 0.2;
    marker.color.r = 1.0; 
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;
    for (const auto &pose : global_path.poses)
    {
        marker.points.push_back(pose.pose.position);
    }
    pub_global_path_.publish(marker);
}

void RvizVisualizer::publishVehiclePose(const geometry_msgs::Pose &pose)
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = my_frame_id; 
    marker.header.stamp = ros::Time::now();
    marker.id = 0;
    marker.type = visualization_msgs::Marker::LINE_LIST; 
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = 0.05; 
    marker.color.r = 0.0;  
    marker.color.g = 0.0;  
    marker.color.b = 1.0;  
    marker.color.a = 1.0;  
    double yaw = tf::getYaw(pose.orientation);

    std::vector<geometry_msgs::Point> local_vertices = calculateVehicleRectVertices();
    std::vector<geometry_msgs::Point> world_vertices;
    for (const auto &local_p : local_vertices)
    {
        geometry_msgs::Point rotated_p = rotatePoint(local_p, yaw);
        geometry_msgs::Point world_p;
        world_p.x = rotated_p.x + pose.position.x;
        world_p.y = rotated_p.y + pose.position.y;
        world_p.z = rotated_p.z + pose.position.z; 
        world_vertices.push_back(world_p);
    }
    marker.points.push_back(world_vertices[0]);
    marker.points.push_back(world_vertices[1]);
    marker.points.push_back(world_vertices[1]);
    marker.points.push_back(world_vertices[2]);
    marker.points.push_back(world_vertices[2]);
    marker.points.push_back(world_vertices[3]);
    marker.points.push_back(world_vertices[3]);
    marker.points.push_back(world_vertices[0]);
    pub_vehicle_.publish(marker);
}

void RvizVisualizer::publishGoalPose(const geometry_msgs::Pose &goal_pose)
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();
    marker.id = 0;
    marker.type = visualization_msgs::Marker::ARROW; 
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = 2; 
    marker.scale.y = 0.15;
    marker.scale.z = 0.3;
    marker.color.r = 1.0;
    marker.color.g = 1.0; 
    marker.color.b = 0.0;
    marker.color.a = 1.0;
    marker.pose = goal_pose;
    pub_goal_.publish(marker);
}
