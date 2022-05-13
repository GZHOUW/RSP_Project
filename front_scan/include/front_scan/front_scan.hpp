#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/search/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/extract_clusters.h>
#include <std_msgs/Bool.h>

class front_scan{
private:
    ros::NodeHandle node_handle;
    ros::Subscriber sub_pointCloud;
    ros::Publisher pub_blocked;
    float width=0.3,distance=1.0,ratio_threshold=0.1;
    
public:
    front_scan(ros::NodeHandle& nh);
    void callback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud);

};
