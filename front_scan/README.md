# front_scan

## Overview

This package is used to detect abstacles in front of the turtlebot. The node will subscribe the `/point_cloud` topic, and publish topic `/front_blocked` which is a `std_msgs::Bool` message. if the value is `True`, it means there is an obstacle in front of the robot.

## Parameters

In `front_scan.launch` there are three parameters you can set:  
assume the robot directs to the Y-axis in robot's frame  
`width`: the area we care about on robot's X-axis(-width,width)  
`distance`: the area we care about on robot's Y-axis (0,diatance)  
`ratio_threshold`: the threshold of ratio `close points`/`points in the interest area`  
if the ratio is greater than `ratio_threshold`, we take there is an obstacle in front of the robot.

## Run Demo

1. Build the package
    
    `cd {workspace_dir}`
    
    `catkin build`
    
2. Run the launch file
    
    `roslaunch front_scan front_scan.launch`