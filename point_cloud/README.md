# point_cloud

## Overview

This package is used to convert `sensor_msgs::LaserScan` data to `pcl::PointCloud<pcl::PointXYZ>` data  
It subscribes `/scan` topic and publishes `/point_cloud` topic.

## Run Demo

1. Build the package
    
    `cd {workspace_dir}`
    
    `catkin build`
    
2. Run the launch file
    
    `roslaunch point_cloud point_cloud.launch`