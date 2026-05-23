#include "lidar_ros/lidar_core.h"
#include <unordered_set>
#include <thread>
#include <std_msgs/Float64MultiArray.h>


LidarCore::LidarCore(ros::NodeHandle &nh){

    this->seg_distance_ = {15, 30, 45, 60};
    this->cluster_distance_ = {0.5, 1.0, 1.5, 2.0, 2.5};

    this->tf_listener_ = new tf::TransformListener();

    // /cur_scan_in_map  /kinect2/sd/points
    this->sub_point_cloud_ = nh.subscribe<sensor_msgs::PointCloud2>("/cur_scan_in_map", 5, &LidarCore::point_cb, this);

    this->pub_filter_pointCloud = nh.advertise<sensor_msgs::PointCloud2>("/lidar/filter_pointCloud", 10);
    this->pub_car_box = nh.advertise<jsk_recognition_msgs::BoundingBox>("/lidar/car_box", 10);
    this->pub_car_big_box = nh.advertise<jsk_recognition_msgs::BoundingBox>("/lidar/car_big_box", 10);
    this->pub_bounding_boxs_ = nh.advertise<jsk_recognition_msgs::BoundingBoxArray>("/lidar/detected_bounding_boxs", 5);
    this->pub_bounding_boxs_map_ = nh.advertise<jsk_recognition_msgs::BoundingBoxArray>("/lidar/map_detected_bounding_boxs", 5);
    this->pub_float_array_msg = nh.advertise<std_msgs::Float64MultiArray>("/lidar/detect_result", 10);
    this->pub_map_float_array_msg = nh.advertise<std_msgs::Float64MultiArray>("/lidar/map_detect_result", 10);
    this->pub_filter_pointCloud_map = nh.advertise<sensor_msgs::PointCloud2>("/lidar/filter_pointCloud_map", 10);


    ros::spin();
}


LidarCore::~LidarCore(){
    // 释放 TransformListener 指针
    delete this->tf_listener_;
}


// 计算点云中的点数
void LidarCore::numPoints( pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud)
{
    std::cout << cloud->points.size() << std::endl;
}


double LidarCore::Q2RPY(geometry_msgs::Quaternion quaternion){
    // 定义一个四元数
    // 注意：定义四元数的顺序是 [w, x, y, z]
    // Eigen::Quaternionf quaternion(-0.5, 0.5, 0.5, -0.5); // [w, x, y, z]   Quaternion: 0.5 -0.5 -0.5 0.5
    // Eigen::Quaternionf quaternion(0.5, -0.5, -0.5, 0.5); // [w, x, y, z]   Quaternion: 0.5 -0.5 -0.5 0.5
    // Eigen::Quaternionf quaternion(0.481964, -0.000117356, -0.00196721, 0.876189);

    Eigen::Quaternionf q;
    q.x() = quaternion.x;
    q.y() = quaternion.y;
    q.z() = quaternion.z;
    q.w() = quaternion.w;

    // 将四元数转换为旋转矩阵
    Eigen::Matrix3f rotation_matrix = q.toRotationMatrix();

    // 从旋转矩阵中提取 RPY 角度（弧度）
    double roll_rad = atan2(rotation_matrix(2, 1), rotation_matrix(2, 2));
    double pitch_rad = asin(-rotation_matrix(2, 0));
    double yaw_rad = atan2(rotation_matrix(1, 0), rotation_matrix(0, 0));

    // 将弧度转换为角度
    double roll_deg = 180 * (roll_rad / M_PI);
    double pitch_deg = 180 * (pitch_rad / M_PI);
    double yaw_deg = 180 * (yaw_rad / M_PI);

    // 输出 RPY 角度（角度）
    // std::cout << "Quaternion: " << q.w() << " " << q.x() << " " << q.y() << " " << q.z() << std::endl;
    // std::cout << "RPY: Roll=" << roll_deg << "°, Pitch=" << pitch_deg << "°, Yaw=" << yaw_deg << "°" << std::endl;

    return yaw_rad;
}


double LidarCore::Q2RPY(double x, double y, double z, double w){
    // 定义一个四元数
    // 注意：定义四元数的顺序是 [w, x, y, z]
    // Eigen::Quaternionf quaternion(-0.5, 0.5, 0.5, -0.5); // [w, x, y, z]   Quaternion: 0.5 -0.5 -0.5 0.5
    // Eigen::Quaternionf quaternion(0.5, -0.5, -0.5, 0.5); // [w, x, y, z]   Quaternion: 0.5 -0.5 -0.5 0.5
    // Eigen::Quaternionf quaternion(0.481964, -0.000117356, -0.00196721, 0.876189);

    Eigen::Quaternionf q;
    q.x() = x;
    q.y() = y;
    q.z() = z;
    q.w() = w;

    // 将四元数转换为旋转矩阵
    Eigen::Matrix3f rotation_matrix = q.toRotationMatrix();

    // 从旋转矩阵中提取 RPY 角度（弧度）
    double roll_rad = atan2(rotation_matrix(2, 1), rotation_matrix(2, 2));
    double pitch_rad = asin(-rotation_matrix(2, 0));
    double yaw_rad = atan2(rotation_matrix(1, 0), rotation_matrix(0, 0));

    // 将弧度转换为角度
    double roll_deg = 180 * (roll_rad / M_PI);
    double pitch_deg = 180 * (pitch_rad / M_PI);
    double yaw_deg = 180 * (yaw_rad / M_PI);

    // 输出 RPY 角度（角度）
    // std::cout << "Quaternion: " << q.w() << " " << q.x() << " " << q.y() << " " << q.z() << std::endl;
    // std::cout << "RPY: Roll=" << roll_deg << "°, Pitch=" << pitch_deg << "°, Yaw=" << yaw_deg << "°" << std::endl;

    return yaw_rad;
}


// 使用StatisticalOutlierRemoval统计学离群点移除过滤器移除噪点
void LidarCore::statistical_removal(pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud_ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr out_cloud_ptr,
                                    int meanK, float StddevMulThresh){

    pcl::StatisticalOutlierRemoval<pcl::PointXYZI> sor;
    sor.setInputCloud(in_cloud_ptr);
    //设置用于平均距离估计的 KD-tree最近邻搜索点的个数
    sor.setMeanK(meanK); //meanK = 50
    // 设置高斯分布标准差阈值系数, 也就是 u+1*sigma,u+2*sigma,u+3*sigma 中的 系数1、2、3 
    sor.setStddevMulThresh(StddevMulThresh); //StddevMulThreshv = 1.0
    // 执行过滤
    sor.filter(*out_cloud_ptr);


    // 使用个相同的过滤器，但是对输出结果取反，则得到那些被过滤掉的点，保存到_outliers.pcd文件
    // sor.setNegative(true);
    // sor.filter();
}


// 对输入的点云进行滤波处理，包括体素网格滤波和感兴趣区域过滤
// 输入参数：点云指针，体素尺寸，感兴趣区域范围（最近点和最远点，相当于长方体的对角顶点）
pcl::PointCloud<pcl::PointXYZI>::Ptr LidarCore::FilterCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, float voxelGridSize, 
                                                            Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint, 
                                                            Eigen::Vector4f car_minPoint, Eigen::Vector4f car_maxPoint){

    // 记录开始执行滤波的时间
    auto startTime = std::chrono::steady_clock::now();

    // 创建一个新的点云对象，用于存储滤波后的结果
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZI>);

    // 创建体素网格滤波器对象并设置参数，对输入点云进行滤波
    pcl::VoxelGrid<pcl::PointXYZI> vox;
    vox.setInputCloud(cloud);
    vox.setLeafSize(voxelGridSize, voxelGridSize, voxelGridSize); //设置体素网格的大小
    vox.filter(*cloud_filtered); //应用了体素网格滤波器，并将滤波后的结果存储在 cloud_filtered 中，通过解引用 * 将智能指针转换为指向点云对象的指针

    // 创建新的点云对象，用于存储感兴趣区域过滤后的结果
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_roi(new pcl::PointCloud<pcl::PointXYZI>);

    // 创建区域过滤器对象并设置过滤区域的边界，并对体素网格滤波后的点云进行区域过滤
    pcl::CropBox<pcl::PointXYZI> roi(true); //true 参数表示将在给定区域内保留点
    roi.setMin(minPoint); //感兴趣区域的最近点和最远点
    roi.setMax(maxPoint);
    roi.setInputCloud(cloud_filtered);
    roi.filter(*cloud_roi);

    // 额外的区域过滤，例如过滤掉自身车辆车顶的点云
    std::vector<int> idxs;
    pcl::CropBox<pcl::PointXYZI> roof(true); 
    roof.setMin(car_minPoint);
    roof.setMax(car_maxPoint);
    roof.setInputCloud(cloud_roi);
    roof.filter(idxs);

    // 存储 idxs 中的点云到 inliers中
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    for(int point: idxs)
    {
        inliers->indices.push_back(point);
    }

    // 创建了一个点云提取器对象 extract，从 cloud_roi点云中剔除 inliers
    pcl::ExtractIndices<pcl::PointXYZI> extract;
    extract.setInputCloud(cloud_roi);
    extract.setIndices(inliers); //设置了要提取的点的索引，即之前存储在 inliers 中的索引集合
    extract.setNegative(true); //设置了提取器的模式，将其设置为反向模式，即提取除了指定索引 inliers 之外的点集
    extract.filter(*cloud_roi);

    // 记录结束滤波过程的时间
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    // 过滤耗时 毫秒
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloud_roi;
}


// 将输入的点云根据给定的点索引分成两部分成两部分，一部分包含地面上的点，另一部分包含障碍物点

std::pair< pcl::PointCloud<pcl::PointXYZI>::Ptr,  pcl::PointCloud<pcl::PointXYZI>::Ptr> LidarCore::SeparateClouds(pcl::PointIndices::Ptr inliers, 
                                                                                                                  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud) 
{
    //创建两个新的点云对象，一个存储地面上的点云，另一个存储障碍物点云

    // 创建一个点云提取器对象
    pcl::ExtractIndices<pcl::PointXYZI> extract;
    // 设置点云提取器的输入点云数据和要提取的点的索引
    extract.setInputCloud(cloud);
    extract.setIndices(inliers); //inliers为地面点云

     pcl::PointCloud<pcl::PointXYZI>::Ptr ground{new pcl::PointCloud<pcl::PointXYZI>()};
    // 提取地面上的点，并将结果存储在 ground 中
    extract.setNegative(false); //设置了提取器的模式为正向模式，这意味着它将仅提取指定索引 inliers 的点集，而不是排除指定索引的点集。
    extract.filter(*ground);
    
     pcl::PointCloud<pcl::PointXYZI>::Ptr obstacles{new pcl::PointCloud<pcl::PointXYZI>()};
    // 提取障碍物点，并将结果存储在 obstacles 中
    extract.setNegative(true); //设置了提取器的模式为反向模式，即提取除了指定索引 inliers 之外的点集
    extract.filter(*obstacles);

    // 创建一个 pair 对象，存储分割结果，其中第一个元素是障碍物点云，第二个元素是地面点云
    std::pair< pcl::PointCloud<pcl::PointXYZI>::Ptr,  pcl::PointCloud<pcl::PointXYZI>::Ptr> segResult(obstacles, ground);
    return segResult;
}


// 对输入的点云进行平面分割，将点云分成平面上的点和非平面点
// 输入参数：点云指针，RANSAC 算法的最大迭代次数，平面模型拟合的距离阈值，除平面外的的点的个数占比(%)
std::pair< pcl::PointCloud<pcl::PointXYZI>::Ptr,  pcl::PointCloud<pcl::PointXYZI>::Ptr> LidarCore::SegmentPlane( pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, 
                                                                                                                 int maxIterations, 
                                                                                                                 float distanceThreshold, 
                                                                                                                 float obstaclePointScale)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // TODO:: Fill in this function to find inliers for the cloud. 填写此函数可找到地面点云的索引 inliers。

    // 创建一个分割器对象
    pcl::SACSegmentation<pcl::PointXYZI> seg;
    // 创建模型系数和地面点索引对象
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());//内点

    // 设置分割器的参数
    seg.setOptimizeCoefficients(true);               //设置了是否优化平面模型的系数
    // Mandatory
    seg.setModelType(pcl::SACMODEL_PLANE);          //设置了使用的模型类型，这里是平面模型
    seg.setMethodType(pcl::SAC_RANSAC);             //设置了使用的方法类型，这里是随机抽样一致性（RANSAC）算法
    seg.setMaxIterations(maxIterations);            //设置了 RANSAC 算法的最大迭代次数
    seg.setDistanceThreshold(distanceThreshold);    //设置了平面模型拟合的距离阈值。该阈值确定了一个点被认为是平面上的点的最大距离，超过这个距离的点将被视为离群点。

    // RANSAC 循环提取多个平面
    size_t allCloudSize = cloud->points.size(); //点云总数量
    std::vector<pcl::ModelCoefficients::Ptr> coefficients_list; //平面模型的系数列表，存放多个平面的系数
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloudPlane(new pcl::PointCloud<pcl::PointXYZI>()), cloud_other(new pcl::PointCloud<pcl::PointXYZI>());
    // 合并后的点云
    pcl::PointCloud<pcl::PointXYZI>::Ptr merged_cloud(new pcl::PointCloud<pcl::PointXYZI>());

    int i = 0;
    // 使用RANSAC方法不断在点云中提取平面，直到除平面外的的点的个数占比不超过全部点云的 10%
    while(cloud->points.size() > obstaclePointScale * allCloudSize)
    {
        seg.setInputCloud(cloud);
        seg.segment(*inliers, *coefficients);            //执行平面分割算法，并将分割后的平面模型的系数存储在 coefficients 中，将被认为是平面上的点的索引存储在 inliers 中
        coefficients_list.push_back(coefficients);

        // 如果未找到平面模型，则输出错误信息
        if (inliers->indices.size() == 0)
        {
            std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
        }

        pcl::ExtractIndices<pcl::PointXYZI> extract;
        extract.setInputCloud(cloud);
        extract.setIndices(inliers);
        extract.setNegative(false);
        extract.filter(*cloudPlane);
        *merged_cloud += *cloudPlane;

        extract.setNegative(true);
        extract.filter(*cloud_other);

        *cloud = *cloud_other;
        ++i;
    }
    std::cout << "number of planes:" << i << std::endl;

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    // std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair< pcl::PointCloud<pcl::PointXYZI>::Ptr,  pcl::PointCloud<pcl::PointXYZI>::Ptr> segResult(cloud_other, merged_cloud);
    return segResult;
}


// 对输入的障碍物点云进行欧几里得聚类，将点云中的障碍物分组成多个簇
// 输入参数：点云指针，邻近搜索的搜索半径，设置一个聚类需要的最少点数目，最多点数目。对于 pdc/data_1 来说，它工作得很好，但在其他用例中，它可能表现不佳
std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> LidarCore::Clustering(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{
    std::cout << "all obstacles cloud->size();" << cloud->size() << std::endl;
    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    // 创建存储聚类结果的向量
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> clusters;

    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles 填写函数，对检测到的障碍物进行欧氏聚类
    
    // 创建 KD 树对象，并设置输入点云数据
    typename pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>);
    tree->setInputCloud(cloud);

    // 创建点云索引对象，用于存储聚类结果的索引
    std::vector<pcl::PointIndices> clusterIndices;

    // 创建欧几里得聚类对象，并设置参数
    pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
    ec.setClusterTolerance(clusterTolerance);   // 设置聚类的距离阈值
    ec.setMinClusterSize(minSize);              // 设置每个聚类的最小点数
    ec.setMaxClusterSize(maxSize);
    ec.setSearchMethod(tree);                   // 设置聚类时使用的搜索方法
    ec.setInputCloud(cloud);
    ec.extract(clusterIndices);                 // 执行欧几里得聚类

    // 根据聚类结果创建点云对象，并存储在 clusters 中
    for(pcl::PointIndices getIndices: clusterIndices)
    {
        typename pcl::PointCloud<pcl::PointXYZI>::Ptr cloudCluster (new pcl::PointCloud<pcl::PointXYZI>);

        // 将每个聚类的点添加到对应的点云对象中
        for(int idx: getIndices.indices)
        {
            cloudCluster->points.push_back(cloud->points[idx]);
        }
        // 在 PCL 中，点云可以是有序的（组织成二维图像形式）或无序的（即一维的点列表）。对于无序点云，宽度表示点的数量，而高度为 1。
        cloudCluster->width = cloudCluster->points.size();
        cloudCluster->height = 1;
        cloudCluster->is_dense = true;//设置点云为密集型点云

        // 将聚类结果添加到 clusters 中
        clusters.push_back(cloudCluster);
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clusters cost: " << elapsedTime.count() << " milliseconds" << std::endl;
    std::cout << "clusters number: " << clusters.size() << std::endl;

    return clusters;
}


std::vector< pcl::PointCloud<pcl::PointXYZI>::Ptr> LidarCore::cluster_by_distance(pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud_ptr, float clusterTolerance, int minSize, int maxSize){
    
    //cluster the pointcloud according to the distance of the points using different thresholds (not only one for the entire pc)
    //in this way, the points farther in the pc will also be clustered

    //0 => 0-15m d=0.5
    //1 => 15-30 d=1
    //2 => 30-45 d=1.6
    //3 => 45-60 d=2.1
    //4 => >60   d=2.6
    ROS_INFO("**********");
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> segment_pc_array(5);

    for (size_t i = 0; i < segment_pc_array.size(); i++)
    {
        pcl::PointCloud<pcl::PointXYZI>::Ptr tmp(new pcl::PointCloud<pcl::PointXYZI>);
        segment_pc_array[i] = tmp;
    }

    for (size_t i = 0; i < in_cloud_ptr->points.size(); i++)
    {
        pcl::PointXYZI current_point;
        current_point.x = in_cloud_ptr->points[i].x;
        current_point.y = in_cloud_ptr->points[i].y;
        current_point.z = in_cloud_ptr->points[i].z;

        float origin_distance = sqrt(pow(current_point.x, 2) + pow(current_point.y, 2));

        // 如果点的距离大于120m, 忽略该点
        if (origin_distance >= 120)
        {
            continue;
        }

        if (origin_distance < seg_distance_[0])
        {
            segment_pc_array[0]->points.push_back(current_point);
        }
        else if (origin_distance < seg_distance_[1])
        {
            segment_pc_array[1]->points.push_back(current_point);
        }
        else if (origin_distance < seg_distance_[2])
        {
            segment_pc_array[2]->points.push_back(current_point);
        }
        else if (origin_distance < seg_distance_[3])
        {
            segment_pc_array[3]->points.push_back(current_point);
        }
        else
        {
            segment_pc_array[4]->points.push_back(current_point);
        }
    }

    std::vector< pcl::PointCloud<pcl::PointXYZI>::Ptr> obstacle_vector;
    for (size_t i = 0; i < segment_pc_array.size(); i++)
    {
        ROS_INFO("cluster by distance ID: %zu", i);
        std::vector< pcl::PointCloud<pcl::PointXYZI>::Ptr> part_obstacle_vector = Clustering(segment_pc_array[i], cluster_distance_[i], minSize, maxSize);
        // 在 obstacle_vector 尾部插入 part_obstacle_vector 的所有元素
        obstacle_vector.insert(obstacle_vector.end(), part_obstacle_vector.begin(), part_obstacle_vector.end());
    }
    ROS_INFO("**********");
    return obstacle_vector;
}
                                                                    


LidarCore::Deteced_Obj LidarCore::computer_all_my_AABB( pcl::PointCloud<pcl::PointXYZI>::Ptr cluster){

    // 计算质心
    int point_num = cluster->points.size();
    float centroid_x_sum = 0, centroid_y_sum = 0, centroid_z_sum = 0;

    for(int i = 0; i < point_num; ++i){
        centroid_x_sum += cluster->points[i].x;
        centroid_y_sum += cluster->points[i].y;
        centroid_z_sum += cluster->points[i].z;
    }

    float centroid_x = centroid_x_sum/point_num;
    float centroid_y = centroid_y_sum/point_num;
    float centroid_z = centroid_z_sum/point_num;

    // 寻找聚类的最小和最大点
    pcl::PointXYZI minPoint, maxPoint;
    pcl::getMinMax3D(*cluster, minPoint, maxPoint);

    // 计算障碍物的中心点
    pcl::PointXYZI center_point;
    center_point.x = (minPoint.x + maxPoint.x)/2;
    center_point.y = (minPoint.y + maxPoint.y)/2;
    center_point.z = (minPoint.z + maxPoint.z)/2;
    std::cout <<"current box center_point:("<< center_point.x << ", " <<center_point.y << ", " << center_point.z << ")" << std::endl;

    // 计算边界框的长宽高
    pcl::PointXYZI length_width_height;
    length_width_height.x = abs(minPoint.x - maxPoint.x);
    length_width_height.y = abs(minPoint.y - maxPoint.y);
    length_width_height.z = abs(minPoint.z - maxPoint.z);
    std::cout <<"current box---length:"<< length_width_height.x << "  width:" <<length_width_height.y << "  height:" << length_width_height.z << std::endl;


    Deteced_Obj obj_info;

    obj_info.min_point_.x = minPoint.x;
    obj_info.min_point_.y = minPoint.y;
    obj_info.min_point_.z = minPoint.z;

    obj_info.max_point_.x = maxPoint.x;
    obj_info.max_point_.y = maxPoint.y;
    obj_info.max_point_.z = maxPoint.z;

    obj_info.centroid_.x = centroid_x;
    obj_info.centroid_.y = centroid_y;
    obj_info.centroid_.z = centroid_z;

    obj_info.bounding_box_.header = this->point_cloud_header_;

    obj_info.bounding_box_.pose.position.x = center_point.x;
    obj_info.bounding_box_.pose.position.y = center_point.y;
    obj_info.bounding_box_.pose.position.z = center_point.z;

    obj_info.bounding_box_.dimensions.x = length_width_height.x;
    obj_info.bounding_box_.dimensions.y = length_width_height.y;
    obj_info.bounding_box_.dimensions.z = length_width_height.z;

    return obj_info;
}


LidarCore::Deteced_Obj LidarCore::computer_all_AABB( pcl::PointCloud<pcl::PointXYZI>::Ptr cluster){

    // 创建惯性矩估算对象，设置输入点云，并进行计算
    pcl::MomentOfInertiaEstimation <pcl::PointXYZI> feature_extractor;
    feature_extractor.setInputCloud(cluster);
    feature_extractor.compute();

    // 获取AABB盒子
    pcl::PointXYZI min_point_AABB;
    pcl::PointXYZI max_point_AABB;
    feature_extractor.getAABB(min_point_AABB, max_point_AABB);

    // 获取质心
    Eigen::Vector3f mass_center;
    feature_extractor.getMassCenter (mass_center);

    // 计算障碍物的中心点
    pcl::PointXYZI position_AABB;
    position_AABB.x = (min_point_AABB.x + max_point_AABB.x)/2;
    position_AABB.y = (min_point_AABB.y + max_point_AABB.y)/2;
    position_AABB.z = (min_point_AABB.z + max_point_AABB.z)/2;
    std::cout <<"current box center_point:("<< position_AABB.x << ", " <<position_AABB.y << ", " << position_AABB.z << ")" << std::endl;

    // 计算边界框的长宽高
    pcl::PointXYZI length_width_height;
    length_width_height.x = abs(min_point_AABB.x - max_point_AABB.x);
    length_width_height.y = abs(min_point_AABB.y - max_point_AABB.y);
    length_width_height.z = abs(min_point_AABB.z - max_point_AABB.z);
    std::cout <<"current box---length:"<< length_width_height.x << "  width:" <<length_width_height.y << "  height:" << length_width_height.z << std::endl;


    Deteced_Obj obj_info;

    obj_info.min_point_.x = min_point_AABB.x;
    obj_info.min_point_.y = min_point_AABB.y;
    obj_info.min_point_.z = min_point_AABB.z;

    obj_info.max_point_.x = max_point_AABB.x;
    obj_info.max_point_.y = max_point_AABB.y;
    obj_info.max_point_.z = max_point_AABB.z;

    obj_info.centroid_.x = mass_center(0);
    obj_info.centroid_.y = mass_center(1);
    obj_info.centroid_.z = mass_center(2);

    obj_info.bounding_box_.header = this->point_cloud_header_;

    obj_info.bounding_box_.pose.position.x = position_AABB.x;
    obj_info.bounding_box_.pose.position.y = position_AABB.y;
    obj_info.bounding_box_.pose.position.z = position_AABB.z;

    obj_info.bounding_box_.dimensions.x = length_width_height.x;
    obj_info.bounding_box_.dimensions.y = length_width_height.y;
    obj_info.bounding_box_.dimensions.z = length_width_height.z;

    return obj_info;
}


LidarCore::Deteced_Obj LidarCore::computer_all_OBB_z( pcl::PointCloud<pcl::PointXYZI>::Ptr cluster){

    // 创建惯性矩估算对象，设置输入点云，并进行计算
    pcl::MomentOfInertiaEstimation <pcl::PointXYZI> feature_extractor;
    feature_extractor.setInputCloud(cluster);
    feature_extractor.compute();

    // 获取质心
    Eigen::Vector3f mass_center;
    feature_extractor.getMassCenter (mass_center);

    // 获取OBB盒子
    pcl::PointXYZI min_point_OBB; //OBB坐标系下
    pcl::PointXYZI max_point_OBB; //OBB坐标系下
    pcl::PointXYZI position_OBB;  //点云坐标系下，即 t(点云_OBB) 
    Eigen::Matrix3f rotational_matrix_OBB; //表示的是 点云数据坐标系 到 OBB坐标系 的坐标变换， 即 R(点云_OBB)
    feature_extractor.getOBB(min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB);


    // 只绕 z 轴旋转
    Eigen::Quaternionf q_OBB(rotational_matrix_OBB);

    double yaw = this->Q2RPY(q_OBB.x(), q_OBB.y(), q_OBB.z(), q_OBB.w()); //弧度制

    // 使用Eigen::AngleAxis创建一个绕Z轴的旋转
    Eigen::AngleAxis<float> rotation_z(yaw, Eigen::Vector3f::UnitZ());

    Eigen::Matrix3f rotation_matrix_z = rotation_z.toRotationMatrix();
    Eigen::Quaternionf q_OBB_z(rotation_matrix_z);


    // 障碍物的中心点
    std::cout <<"current box center_point:("<< position_OBB.x << ", " <<position_OBB.y << ", " << position_OBB.z << ")" << std::endl;

    // 计算边界框的长宽高
    pcl::PointXYZI length_width_height;
    length_width_height.x = abs(min_point_OBB.x - max_point_OBB.x);
    length_width_height.y = abs(min_point_OBB.y - max_point_OBB.y);
    length_width_height.z = abs(min_point_OBB.z - max_point_OBB.z);
    std::cout <<"current box---length:"<< length_width_height.x << "  width:" <<length_width_height.y << "  height:" << length_width_height.z << std::endl;


    Deteced_Obj obj_info;

    obj_info.min_point_.x = min_point_OBB.x;
    obj_info.min_point_.y = min_point_OBB.y;
    obj_info.min_point_.z = min_point_OBB.z;

    obj_info.max_point_.x = max_point_OBB.x;
    obj_info.max_point_.y = max_point_OBB.y;
    obj_info.max_point_.z = max_point_OBB.z;

    obj_info.centroid_.x = mass_center(0);
    obj_info.centroid_.y = mass_center(1);
    obj_info.centroid_.z = mass_center(2);

    obj_info.bounding_box_.header = this->point_cloud_header_;

 
    obj_info.bounding_box_.pose.orientation.x = q_OBB_z.x();
    obj_info.bounding_box_.pose.orientation.y = q_OBB_z.y();
    obj_info.bounding_box_.pose.orientation.z = q_OBB_z.z();
    obj_info.bounding_box_.pose.orientation.w = q_OBB_z.w();

    obj_info.bounding_box_.pose.position.x = position_OBB.x;
    obj_info.bounding_box_.pose.position.y = position_OBB.y;
    obj_info.bounding_box_.pose.position.z = position_OBB.z;

    obj_info.bounding_box_.dimensions.x = length_width_height.x;
    obj_info.bounding_box_.dimensions.y = length_width_height.y;
    obj_info.bounding_box_.dimensions.z = length_width_height.z;

    return obj_info;
}


LidarCore::Deteced_Obj LidarCore::computer_all_my_OBB_z( pcl::PointCloud<pcl::PointXYZI>::Ptr cluster){

    // 初始化最大和最小Z值为可能的极限值  
    float min_z = std::numeric_limits<float>::max();  
    float max_z = std::numeric_limits<float>::lowest();  
    
    // 遍历点云中的所有点  
    for (auto& point : *cluster)  
    {  
        // 更新最小和最大Z值  
        if (point.z < min_z)  
            min_z = point.z;  
        if (point.z > max_z)  
            max_z = point.z;  
        // 投影到XY平面（简化处理，将Z坐标设为0） 
        point.z = 0; 
    }


    // 创建惯性矩估算对象，设置输入点云，并进行计算
    // 此时，点云为同一平面内, 只有绕Z轴的旋转
    pcl::MomentOfInertiaEstimation <pcl::PointXYZI> feature_extractor;
    feature_extractor.setInputCloud(cluster);
    feature_extractor.compute();

    // 获取OBB盒子
    pcl::PointXYZI min_point_OBB; //OBB坐标系下
    pcl::PointXYZI max_point_OBB; //OBB坐标系下
    pcl::PointXYZI position_OBB;  //点云坐标系下，即 t(点云_OBB) 
    Eigen::Matrix3f rotational_matrix_OBB; //表示的是 点云数据坐标系 到 OBB坐标系 的坐标变换， 即 R(点云_OBB)
    feature_extractor.getOBB(min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB);
    Eigen::Quaternionf q_myOBB(rotational_matrix_OBB);


    ROS_INFO("min_point_OBB(x, y, z): %f, %f, %f", min_point_OBB.x, min_point_OBB.y, min_point_OBB.z);
    ROS_INFO("max_point_OBB(x, y, z): %f, %f, %f", max_point_OBB.x, max_point_OBB.y, max_point_OBB.z);
    ROS_INFO("position_OBB(x, y, z): %f, %f, %f", position_OBB.x, position_OBB.y, position_OBB.z);
    double angle_y = this->Q2RPY(q_myOBB.x(), q_myOBB.y(), q_myOBB.z(), q_myOBB.w());
    ROS_INFO("angle_y: %f", angle_y);

    

    // 计算3D下障碍物的各点
    min_point_OBB.z = min_z;;
    max_point_OBB.z = max_z;
    position_OBB.z = (min_z + max_z)/2;
    std::cout <<"update box Z(position, min, max):  "<< min_point_OBB.z << "  " << max_point_OBB.z << "  " << position_OBB.z << std::endl;


    // 计算3D下边界框的长宽高
    pcl::PointXYZI length_width_height;
    length_width_height.x = abs(min_point_OBB.x - max_point_OBB.x);
    length_width_height.y = abs(min_point_OBB.y - max_point_OBB.y);
    length_width_height.z = abs(min_point_OBB.z - max_point_OBB.z);
    std::cout <<"current box---length:"<< length_width_height.x << "  width:" <<length_width_height.y << "  height:" << length_width_height.z << std::endl;


    Deteced_Obj obj_info;

    obj_info.min_point_.x = min_point_OBB.x;
    obj_info.min_point_.y = min_point_OBB.y;
    obj_info.min_point_.z = min_point_OBB.z;
    obj_info.max_point_.x = max_point_OBB.x;
    obj_info.max_point_.y = max_point_OBB.y;
    obj_info.max_point_.z = max_point_OBB.z;


    obj_info.bounding_box_.header = this->point_cloud_header_;

    obj_info.bounding_box_.pose.position.x = position_OBB.x;
    obj_info.bounding_box_.pose.position.y = position_OBB.y;
    obj_info.bounding_box_.pose.position.z = position_OBB.z;

    obj_info.bounding_box_.pose.orientation.x = q_myOBB.x();
    obj_info.bounding_box_.pose.orientation.y = q_myOBB.y();
    obj_info.bounding_box_.pose.orientation.z = q_myOBB.z();
    obj_info.bounding_box_.pose.orientation.w = q_myOBB.w();

    obj_info.bounding_box_.dimensions.x = length_width_height.x;
    obj_info.bounding_box_.dimensions.y = length_width_height.y;
    obj_info.bounding_box_.dimensions.z = length_width_height.z;

    return obj_info;
}



// 传入的参数是单个障碍物的 pose，即 T(body_OBB)
std::pair<geometry_msgs::PoseStamped, tf::StampedTransform> LidarCore::body_to_map(geometry_msgs::Pose pose){

    // 转换为 PointStamped 消息
    geometry_msgs::PoseStamped body_pose_stamped;
    body_pose_stamped.pose.position = pose.position;
    body_pose_stamped.pose.orientation = pose.orientation;
    body_pose_stamped.header.frame_id = "body"; // 设置坐标系
    body_pose_stamped.header.stamp = this->point_cloud_header_.stamp; // 设置时间戳 ros::Time::now()

    // 函数传入参数：geometry_msgs::PointStamped 是 几何点 (geometry_msgs::Point) 和时间戳 (std_msgs::Header)
    // odom  map
    bool result = this->tf_listener_->waitForTransform("map", body_pose_stamped.header.frame_id, 
                                                               body_pose_stamped.header.stamp, ros::Duration(1, 0));
    if(result == false){
        ROS_WARN("tf error!");
    }

    // 注意问题原因: 发布的坐标系相对关系未拿到坐标点却开始发生转换
    geometry_msgs::PoseStamped map_pose_stamped;
    tf::StampedTransform Transform_map_body;

    try{
        this->tf_listener_->transformPose("map", body_pose_stamped, map_pose_stamped);
        std::cout << "map_position.header.frame_id: " << map_pose_stamped.header.frame_id << std::endl;
        
        // 获取 T(map_body), 其中 t(map_body)为车辆在世界坐标下下的中心坐标
        this->tf_listener_->lookupTransform("map", body_pose_stamped.header.frame_id, 
                                                    body_pose_stamped.header.stamp, Transform_map_body) ;
        
    } 
    catch (const std::exception &ex) {
        // 处理异常的代码
        ROS_WARN("catch tf: %s", ex.what());
    }

    std::pair<geometry_msgs::PoseStamped, tf::StampedTransform> result_pair(map_pose_stamped, Transform_map_body);
    return result_pair;
}


LidarCore::Deteced_Obj LidarCore::box_body_to_map(Deteced_Obj body_box){
    // 转换为 PointStamped 消息
    geometry_msgs::PoseStamped body_pose_stamped;
    body_pose_stamped.pose.position = body_box.bounding_box_.pose.position;       // t(点云_OBB) 
    body_pose_stamped.pose.orientation = body_box.bounding_box_.pose.orientation; // R(点云_OBB)
    body_pose_stamped.header = this->point_cloud_header_;


    // 函数传入参数：geometry_msgs::PointStamped 是 几何点 (geometry_msgs::Point) 和时间戳 (std_msgs::Header)
    // odom  map
    bool result = this->tf_listener_->waitForTransform("map", body_pose_stamped.header.frame_id, 
                                                               body_pose_stamped.header.stamp, ros::Duration(1, 0));
    if(result == false){
        ROS_WARN("tf error!");
    }

    // 注意问题原因: 发布的坐标系相对关系未拿到坐标点却开始发生转换
    geometry_msgs::PoseStamped map_pose_stamped;


    try{
        this->tf_listener_->transformPose("map", body_pose_stamped, map_pose_stamped);
        std::cout << "map_position.header.frame_id: " << map_pose_stamped.header.frame_id << std::endl;
        
    } 
    catch (const std::exception &ex) {
        // 处理异常的代码
        ROS_WARN("catch tf: %s", ex.what());
    }

    Deteced_Obj obj;

    // map 坐标系
    obj.bounding_box_.header.seq = this->point_cloud_header_.seq;
    obj.bounding_box_.header.frame_id = "map";
    obj.bounding_box_.header.stamp = this->point_cloud_header_.stamp;

    obj.bounding_box_.pose.position.x = map_pose_stamped.pose.position.x;
    obj.bounding_box_.pose.position.y = map_pose_stamped.pose.position.y;
    obj.bounding_box_.pose.position.z = map_pose_stamped.pose.position.z;

    obj.bounding_box_.pose.orientation.x = map_pose_stamped.pose.orientation.x;
    obj.bounding_box_.pose.orientation.y = map_pose_stamped.pose.orientation.y;
    obj.bounding_box_.pose.orientation.z = map_pose_stamped.pose.orientation.z;
    obj.bounding_box_.pose.orientation.w = map_pose_stamped.pose.orientation.w;

    obj.bounding_box_.dimensions.x = body_box.bounding_box_.dimensions.x;
    obj.bounding_box_.dimensions.y = body_box.bounding_box_.dimensions.y;
    obj.bounding_box_.dimensions.z = body_box.bounding_box_.dimensions.z;

    return obj;
}


void LidarCore::publish_car(){
    // 车辆本体
    jsk_recognition_msgs::BoundingBox car_box;

    double car_length = 1.8;              // 车辆长度
    double car_width = 0.9;             // 车辆宽度

    car_box.header = this->point_cloud_header_; //body
    car_box.pose.position.x = -car_length/2;
    car_box.pose.position.y = 0;
    car_box.pose.position.z = 0;

    car_box.dimensions.x = car_length;
    car_box.dimensions.y = car_width;
    car_box.dimensions.z = 0.1;

    this->pub_car_box.publish(car_box);
}


void LidarCore::publish_car_big(){
    // 车辆本体
    jsk_recognition_msgs::BoundingBox car_big_box;


    double ago_distance_th = 1;       // 前方距离阈值
    double after_distance_th = 0.5;     // 后方距离阈值
    double left_distance_th = 0.1;      // 左侧距离阈值
    double right_distance_th = 0.1;     // 右侧距离阈值
    double car_length = 1.8;              // 车辆长度
    double car_width = 0.9;             // 车辆宽度

    double car_center_x = (ago_distance_th - after_distance_th)/2 - car_length/2;
    double car_center_y = (left_distance_th - right_distance_th)/2;
    double car_length_update = car_length + ago_distance_th + after_distance_th; 
    double car_width_update = car_width + left_distance_th + right_distance_th; 


    car_big_box.header = this->point_cloud_header_; //body
    car_big_box.pose.position.x = car_center_x;
    car_big_box.pose.position.y = car_center_y;
    car_big_box.pose.position.z = 0;

    car_big_box.dimensions.x = car_length_update;
    car_big_box.dimensions.y = car_width_update;
    car_big_box.dimensions.z = 0.1;

    this->pub_car_big_box.publish(car_big_box);
}


void LidarCore::point_cb(const sensor_msgs::PointCloud2ConstPtr &msg){

    ROS_INFO("------------------------");
    auto startTime = std::chrono::steady_clock::now();

    // 参数
    // filter
    int meanK = 0;
    float StddevMulThreshv = 0, voxelGridSize = 0;
    float roi_x_min = 0, roi_x_max = 0, roi_y_min = 0, roi_y_max = 0, roi_z_min = 0, roi_z_max = 0;
    float car_x_min = 0, car_x_max = 0, car_y_min = 0, car_y_max = 0, car_z_min = 0, car_z_max = 0;
    // Plane Segmentation
    int RANSAC_max_iterations = 0;
    float plane_distance_Threshold = 0; //平面模型拟合的距离阈值
    float obstaclePointScale = 0;
    // Cluster
    float cluster_Tolerance = 0;       // 邻近搜索的搜索半径。在这个距离之内的点被视为同一簇的一部分。
    int min_cluster_size = 0, max_cluster_size = 0;
    
    // 从参数服务器读取参数值赋给变量（包括launch文件和launch读取的yaml文件中的参数）
    ros::param::param<int>("filter/meanK", meanK, 0);
    ros::param::param<float>("filter/StddevMulThreshv", StddevMulThreshv, 0);
    ros::param::param<float>("filter/voxelGridSize", voxelGridSize, 0);
    ros::param::param<float>("filter/roi/roi_x_min", roi_x_min, 0);
    ros::param::param<float>("filter/roi/roi_x_max", roi_x_max, 0);
    ros::param::param<float>("filter/roi/roi_y_min", roi_y_min, 0);
    ros::param::param<float>("filter/roi/roi_y_max", roi_y_max, 0);
    ros::param::param<float>("filter/roi/roi_z_min", roi_z_min, 0);
    ros::param::param<float>("filter/roi/roi_z_max", roi_z_max, 0);

    ros::param::param<float>("filter/car/car_x_min", car_x_min, 0);
    ros::param::param<float>("filter/car/car_x_max", car_x_max, 0);
    ros::param::param<float>("filter/car/car_y_min", car_y_min, 0);
    ros::param::param<float>("filter/car/car_y_max", car_y_max, 0);
    ros::param::param<float>("filter/car/car_z_min", car_z_min, 0);
    ros::param::param<float>("filter/car/car_z_max", car_z_max, 0);

    ros::param::param<int>("plane_seg/RANSAC_max_iterations", RANSAC_max_iterations, 0);
    ros::param::param<float>("plane_seg/plane_distance_Threshold", plane_distance_Threshold, 0);
    ros::param::param<float>("plane_seg/obstaclePointScale", obstaclePointScale, 0);

    ros::param::param<float>("cluster/cluster_Tolerance", cluster_Tolerance, 0);
    ros::param::param<int>("cluster/cluster_size/min_cluster_size", min_cluster_size, 0);
    ros::param::param<int>("cluster/cluster_size/max_cluster_size", max_cluster_size, 0);
    

    // 将点云数值从 camera_init 坐标系转换到 body 坐标系，msg为 camera_init 坐标系下的点云
    // ros::Duration(1, 0) 表示一个时长为 1 秒的时间间隔。这个时间间隔通常用于在 ROS 中等待某个操作完成，例如等待 TF（变换）信息的获取完成或者等待某个操作的执行完成。
    // tf_listener->waitForTransform 是用于等待从源坐标系（由 (*msg).header.frame_id 指定）到目标坐标系（body）的转换可用的函数。
    bool result = this->tf_listener_->waitForTransform("body", (*msg).header.frame_id, (*msg).header.stamp, ros::Duration(1, 0));
    if(result == false){
        ROS_WARN("Transform from frame %s to /body not found", (*msg).header.frame_id.c_str());
        return;
    }

    sensor_msgs::PointCloud2 pc_body; //用于存放的 body 坐标系下的点云
    pcl_ros::transformPointCloud("body", *msg, pc_body, *(this->tf_listener_)); //"/body": 这是目标（或称为“目标帧”）的TF（Transform）框架名称

    this->point_cloud_header_ = pc_body.header;

    ROS_WARN("(*msg).header.seq: %u", (*msg).header.seq);
    ROS_WARN("(*msg).header.frame_id: %s", (*msg).header.frame_id.c_str()); //camera_init
    ROS_WARN("(*msg).header.stamp: %f", (*msg).header.stamp.toSec());

    ROS_WARN("this->point_cloud_header_.seq: %u", this->point_cloud_header_.seq);
    ROS_WARN("this->point_cloud_header_.frame_id: %s", this->point_cloud_header_.frame_id.c_str()); //body
    ROS_WARN("this->point_cloud_header_.stamp: %f", this->point_cloud_header_.stamp.toSec());


    // 将点云格式由ROS格式转换为PCL格式
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_src(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(pc_body, *cloud_src); //cloud_src 为 body 坐标系下的 PCL 格式点云

    // 创建存储统计学离群点移除后的点云的指针
    pcl::PointCloud<pcl::PointXYZI>::Ptr removalCloud(new pcl::PointCloud<pcl::PointXYZI>); //removalCloud 为 body 坐标系下过虑后的 PCL 格式点云
    this->statistical_removal(cloud_src, removalCloud, meanK, StddevMulThreshv);

    // 创建存储滤波后的点云的指针
    pcl::PointCloud<pcl::PointXYZI>::Ptr filterCloud(new pcl::PointCloud<pcl::PointXYZI>); //filterCloud 为 body 坐标系下过虑后的 PCL 格式点云
    // 过滤输入点云数据，获取滤波后的点云
    filterCloud = this->FilterCloud(removalCloud, voxelGridSize , 
                                    Eigen::Vector4f (roi_x_min, roi_y_min, roi_z_min, 1), 
                                    Eigen::Vector4f (roi_x_max, roi_y_max, roi_z_max, 1),
                                    Eigen::Vector4f (car_x_min, car_y_min, car_z_min, 1), 
                                    Eigen::Vector4f (car_x_max, car_y_max, car_z_max, 1));

    // rviz 可视化滤波后点云，body坐标系
    sensor_msgs::PointCloud2 Filter_pointCloud;
    pcl::toROSMsg(*filterCloud, Filter_pointCloud);
    Filter_pointCloud.header = this->point_cloud_header_;
    this->pub_filter_pointCloud.publish(Filter_pointCloud);


/////////////////////////////////////////////////////////////

// TODO
    // 将点云数值从 body 坐标系转换到 map 坐标系
    bool result_map = this->tf_listener_->waitForTransform("map", Filter_pointCloud.header.frame_id, Filter_pointCloud.header.stamp, ros::Duration(1, 0));
    if(result_map == false){
        ROS_WARN("Transform from frame %s to /map not found", Filter_pointCloud.header.frame_id.c_str());
        return;
    }

    sensor_msgs::PointCloud2 pc_map; //用于存放的 map 坐标系下的点云
    pcl_ros::transformPointCloud("map", Filter_pointCloud, pc_map, *(this->tf_listener_)); //"/map": 这是目标（或称为“目标帧”）的TF（Transform）框架名称
    this->pub_filter_pointCloud_map.publish(pc_map);




    // 从过滤后的点云中分割出道路平面点云和障碍物点云。 segmentCloud.first为障碍物点云，segmentCloud.second为地面点云
    // std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloud = this->SegmentPlane(filterCloud, RANSAC_max_iterations, plane_distance_Threshold, obstaclePointScale);
    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloud(filterCloud, filterCloud);

    // 对障碍物点云进行聚类
    // std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = this->Clustering(segmentCloud.first, cluster_Tolerance, min_cluster_size, max_cluster_size);
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = this->cluster_by_distance(segmentCloud.first, cluster_Tolerance, min_cluster_size, max_cluster_size);


    // 发布 障碍物 消息，包括中心点，长宽高，朝向角(弧度制)
    // 创建 Float64MultiArray 消息对象，
    std_msgs::Float64MultiArray body_float_array_msg; //10个数据为一组
    body_float_array_msg.data.clear();
    std_msgs::Float64MultiArray map_float_array_msg; //10个数据为一组
    map_float_array_msg.data.clear();
    // 发布 Box Array消息
    jsk_recognition_msgs::BoundingBoxArray body_box_array;
    jsk_recognition_msgs::BoundingBoxArray map_box_array;

    // 遍历每个簇
    int clusterId = 0;
    for(pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : cloudClusters)
    {
        // 打印每个簇的点数
        std::cout << "current cluster ID:" << clusterId << std::endl;
        std::cout << "current cluster size:";
        this->numPoints(cluster);

        const Deteced_Obj obj_info_body = this->computer_all_my_OBB_z(cluster);
        geometry_msgs::Quaternion q_msgs = obj_info_body.bounding_box_.pose.orientation;
        ROS_WARN("q_body:(x,y,z,w): %f,%f,%f,%f", q_msgs.x, q_msgs.y, q_msgs.z, q_msgs.w);
        double angle_y = this->Q2RPY(q_msgs); //障碍物绕雷达坐标系z轴旋转的弧度
        ROS_WARN("angle_y_body:%f", angle_y);

        // rviz显示
        body_box_array.boxes.push_back(obj_info_body.bounding_box_); //body坐标系下
        
        // 将点云数据中的 x、y、z 坐标依次放入消息中，为 body坐标系下坐标
        body_float_array_msg.data.push_back(static_cast<double>(obj_info_body.bounding_box_.pose.position.x)); //x坐标
        body_float_array_msg.data.push_back(static_cast<double>(obj_info_body.bounding_box_.pose.position.y));
        body_float_array_msg.data.push_back(static_cast<double>(obj_info_body.bounding_box_.pose.position.z));
        body_float_array_msg.data.push_back(static_cast<double>(0));                                           //x速度 
        body_float_array_msg.data.push_back(static_cast<double>(0));
        body_float_array_msg.data.push_back(static_cast<double>(0));
        body_float_array_msg.data.push_back(static_cast<double>(obj_info_body.bounding_box_.dimensions.x));    //x长度
        body_float_array_msg.data.push_back(static_cast<double>(obj_info_body.bounding_box_.dimensions.y));
        body_float_array_msg.data.push_back(static_cast<double>(obj_info_body.bounding_box_.dimensions.z));
        body_float_array_msg.data.push_back(static_cast<double>(angle_y));                                     //障碍物绕 雷达坐标系z轴 旋转的弧度，障碍物朝向角

        

        const Deteced_Obj obj_info_map = this->box_body_to_map(obj_info_body);
        geometry_msgs::Quaternion q_map_msgs = obj_info_map.bounding_box_.pose.orientation;
        ROS_WARN("q_map:(x,y,z,w): %f,%f,%f,%f", q_map_msgs.x, q_map_msgs.y, q_map_msgs.z, q_map_msgs.w);
        double angle_map_y = this->Q2RPY(q_map_msgs); //障碍物绕地图坐标系z轴旋转的弧度
        ROS_WARN("angle_y_map:%f", angle_map_y);

        // rviz显示
        map_box_array.boxes.push_back(obj_info_map.bounding_box_); //body坐标系下

        // 单个障碍物 在 世界坐标系下的信息
        map_float_array_msg.data.push_back(static_cast<double>(obj_info_map.bounding_box_.pose.position.x));  //x坐标
        map_float_array_msg.data.push_back(static_cast<double>(obj_info_map.bounding_box_.pose.position.y));
        map_float_array_msg.data.push_back(static_cast<double>(obj_info_map.bounding_box_.pose.position.z));
        map_float_array_msg.data.push_back(static_cast<double>(0));                                           //x速度 
        map_float_array_msg.data.push_back(static_cast<double>(0));
        map_float_array_msg.data.push_back(static_cast<double>(0));
        map_float_array_msg.data.push_back(static_cast<double>(obj_info_map.bounding_box_.dimensions.x));     //x长度
        map_float_array_msg.data.push_back(static_cast<double>(obj_info_map.bounding_box_.dimensions.y));
        map_float_array_msg.data.push_back(static_cast<double>(obj_info_map.bounding_box_.dimensions.z));
        map_float_array_msg.data.push_back(static_cast<double>(angle_map_y));                                //障碍物绕 地图坐标系z轴 旋转的弧度，障碍物朝向角
    

        ++clusterId;
    }

    this->pub_float_array_msg.publish(body_float_array_msg);
    this->pub_map_float_array_msg.publish(map_float_array_msg);

    body_box_array.header = this->point_cloud_header_; //body坐标系下
    this->pub_bounding_boxs_.publish(body_box_array);
    map_box_array.header.frame_id = "map"; //map坐标系下
    this->pub_bounding_boxs_map_.publish(map_box_array);
    
    this->publish_car();
    this->publish_car_big(); //body

    // 使程序休眠 20 毫秒，以控制点云的显示速度
    // std::this_thread::sleep_for(std::chrono::milliseconds(20));

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "All cost " << elapsedTime.count() << " milliseconds " << std::endl;
}
