#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <laser_geometry/laser_geometry.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>


class Laser{
private:
    ros::NodeHandle node_handle;
    ros::Subscriber sub_laserScan;
    ros::Publisher pub_pointCloud;
    laser_geometry::LaserProjection projector;

public:
    Laser(ros::NodeHandle& nh);

    void lidar_callback(const sensor_msgs::LaserScan::ConstPtr& scan_msg);
}; 