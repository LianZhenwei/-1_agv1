#ifndef LIDAR_ROS_H
#define LIDAR_ROS_H

#include <ros/ros.h>

#include <tf/transform_listener.h>

#include <pcl/common/common.h>

#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>

#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/conditional_euclidean_clustering.h>
#include <pcl/segmentation/extract_clusters.h>

#include <pcl/search/organized.h>
#include <pcl/search/kdtree.h>

#include <pcl/features/moment_of_inertia_estimation.h>

#include <std_msgs/Header.h>
#include <sensor_msgs/PointCloud2.h>
#include <jsk_recognition_msgs/BoundingBox.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>


class LidarCore
{
private:
    struct Deteced_Obj
    {
        jsk_recognition_msgs::BoundingBox bounding_box_;//包括中心点，位姿R，长宽高
        pcl::PointXYZI min_point_;
        pcl::PointXYZI max_point_;
        pcl::PointXYZI centroid_;//质心
    };
    
    ros::Subscriber sub_point_cloud_;

    ros::Publisher pub_car_box;
    ros::Publisher pub_car_big_box;
    ros::Publisher pub_bounding_boxs_;
    ros::Publisher pub_bounding_boxs_map_;
    ros::Publisher pub_float_array_msg;
    ros::Publisher pub_map_float_array_msg;
    ros::Publisher pub_filter_pointCloud;
    ros::Publisher pub_filter_pointCloud_map;

    std_msgs::Header point_cloud_header_;

    tf::TransformListener *tf_listener_;

    std::vector<double> seg_distance_, cluster_distance_;

    void numPoints( pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud);

    void statistical_removal(pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud_ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr out_cloud_ptr,
                             int meanK, float StddevMulThresh);
    
    pcl::PointCloud<pcl::PointXYZI>::Ptr FilterCloud( pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, float voxelGridSize, 
                                                      Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint, 
                                                      Eigen::Vector4f car_minPoint, Eigen::Vector4f car_maxPoint);
    
    std::pair< pcl::PointCloud<pcl::PointXYZI>::Ptr,  pcl::PointCloud<pcl::PointXYZI>::Ptr> SeparateClouds(pcl::PointIndices::Ptr inliers, 
                                                                                                           pcl::PointCloud<pcl::PointXYZI>::Ptr cloud);
    
    std::pair< pcl::PointCloud<pcl::PointXYZI>::Ptr,  pcl::PointCloud<pcl::PointXYZI>::Ptr> SegmentPlane( pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, 
                                                                                                          int maxIterations, 
                                                                                                          float distanceThreshold, 
                                                                                                          float obstaclePointScale);


    std::vector< pcl::PointCloud<pcl::PointXYZI>::Ptr> cluster_by_distance(pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud_ptr, 
                                                                           float clusterTolerance, int minSize, int maxSize);


    std::vector< pcl::PointCloud<pcl::PointXYZI>::Ptr> Clustering( pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, 
                                                                   float clusterTolerance, 
                                                                   int minSize, int maxSize);

    Deteced_Obj computer_all_my_AABB( pcl::PointCloud<pcl::PointXYZI>::Ptr cluster);
    Deteced_Obj computer_all_AABB( pcl::PointCloud<pcl::PointXYZI>::Ptr cluster);
    Deteced_Obj computer_all_OBB_z( pcl::PointCloud<pcl::PointXYZI>::Ptr cluster);
    Deteced_Obj computer_all_my_OBB_z( pcl::PointCloud<pcl::PointXYZI>::Ptr cluster);

    std::pair<geometry_msgs::PoseStamped, tf::StampedTransform> body_to_map(geometry_msgs::Pose pose);
    Deteced_Obj box_body_to_map(Deteced_Obj body_box);

    // 这两个函数的变量名有点问题
    void publish_car();
    void publish_car_big();

    void point_cb(const sensor_msgs::PointCloud2ConstPtr &msg);

    double Q2RPY(geometry_msgs::Quaternion quaternion);
    double Q2RPY(double x, double y, double z, double w);

public:
    LidarCore(ros::NodeHandle &nh);
    ~LidarCore();
};


#endif