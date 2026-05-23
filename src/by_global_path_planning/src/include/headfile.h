#ifndef _headfile_h
#define _headfile_h
using namespace std;
//ROS功能包下所用的文件
#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include "geometry_msgs/PoseStamped.h"

//c++库
#include <fstream>
#include "vector"
#include <iostream>

//自己所写的文件
#include "by_global_path_planning/Path.h"
#include "by_global_path_planning/PathPoint.h"
#include "Dijkstra.h"
#include "Box2d.h"
#include "rviz.h"


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
               // cout<<"//////road.id//////"<<road.id<<endl;
                continue;
            }
            double l ;
            double r;
            if(line2 == "road_l"){
                ss>>l;
                continue;
            }
           
           if(line2 == "road_r"){
                ss>>r;
                continue;
            }

            if (line2 == "point")
            {
                record_points = true;
                continue;
            }

            if (line2 != "pre" && line2 != "beh" && record_points)
            {
             Point  point ;
             point.l = l;
             point.r = r;
             ss >> point.x >> point.y>>point.theta;
             road.road_points.push_back(point); 
             continue;
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
                      // cout<<"///ssss.id////"<<id<<endl;
                       road.pre.push_back(id);
                   }
                }
               // cout<<"///ss.count////"<<count<<endl;

                record_points = false;
               // cout<<"///road.pre.size///"<<road.pre.size()<<endl;
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
                      // cout<<"///ssss.id////"<<id<<endl;
                       road.beh.push_back(id);
                   }
                }
                //cout<<"///ss.count////"<<count<<endl;
                
                record_points = false;
               // cout<<"///road.beh.size///"<<road.beh.size()<<endl;
                roads.push_back(road); 
                road = {0};               
                continue;  
            }
        }   
    }
   // cout<<"///roads///"<<roads.size()<<endl;
    fs.close();
    return roads;
}

//加载txt中路径
 std::vector<Road> load_direction_Road2(const std::string fileName)
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
            //  point.x=-point.x;
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




#endif