#include <human_detection/lidar_sub.hpp>

Laser::Laser(ros::NodeHandle& nh){
    node_handle = nh;
    // call callback function in constructor
    sub_laserScan = nh.subscribe("/base_scan", 1000, &Laser::lidar_callback, this);
    pub_pointCloud = nh.advertise<pcl::PointCloud<pcl::PointXYZ>>("/point_cloud", 1);
}

void Laser::lidar_callback(const sensor_msgs::LaserScan::ConstPtr& scan_msg){
    // convert received sensor_msgs::LaserScan data to sensor_msgs::PointCloud2
    sensor_msgs::PointCloud2 sensor_cloud;
    projector.projectLaser(*scan_msg, sensor_cloud);

    // convert sensor_msgs::PointCloud2 to pcl::PCLPointCloud2
    pcl::PCLPointCloud2 pcl_cloud2;
    pcl_conversions::toPCL(sensor_cloud , pcl_cloud2);

    // convert pcl::PCLPointCloud2 to pcl::PointCloud<pcl::PointXYZ>
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(pcl_cloud2, *pcl_cloud);
    
    // publish pcl::PointCloud<pcl::PointXYZ>
    pub_pointCloud.publish(pcl_cloud);
}