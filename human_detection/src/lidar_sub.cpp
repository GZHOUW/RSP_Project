#include <human_detection/lidar_sub.hpp>

Laser::Laser(ros::NodeHandle& nh){
    node_handle = nh;
    // call callback function in constructor
    laser_sub = nh.subscribe("base_scan", 1000, &Laser::lidar_callback, this);
}

void Laser::lidar_callback(const sensor_msgs::LaserScan::ConstPtr& scan_msg){
    // convert received sensor_msgs::LaserScan data to sensor_msgs::PointCloud2
    sensor_msgs::PointCloud2 cloud;
    projector.projectLaser(*scan_msg, cloud);

    // convert sensor_msgs::PointCloud2 to pcl::PointCloud2
    //pcl::PCLPointCloud2 pcl_pc2;
    pcl::PCLPointCloud2 pcl_cloud;
    pcl_conversions::toPCL(cloud , pcl_cloud);
    pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(pcl_cloud, *temp_cloud);
    
    // publish pcl::PointCloud2
}