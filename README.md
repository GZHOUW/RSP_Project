# RSP_Project 
## Guangwei_Zhou(gzhou11@jh.edu) 
## Jiahe_Xu(jxu109@jh.edu)

In this project, use `gazebo` for simulation, `move_base` package for planning. We also used clusting methods with cloudpoint data for object detection.   

In `follow_path` package we assigned a path and feed the path to `move_base`, then the turtlebot will follow the trajectory if it is reachable.   

In `move_turtlebot` package we let turtlebot randomly move in one of the assigned directions according to 'local_costmap' and laser sensor.  

In `turtlebot_planner` package we used RTT method and implemented a `move_base` plugin for planning.

Package `front_scan` and `object_detection` use laser sensor to detect obstacles with clusting methods(KD-tree)  

Package `cloud_point` is used to convert original laser data to point cloud data.  

In our tests, we used perfect maps and map generated by `gmapping`, in the demo (Readme files in each package), we only use perfect map for better result in demo.
  
packages developed by us:  
`turtlebot_planner`  
`follow_path`  
`move_turtlebot`  
`front_scan`  
`object_detection`  
`cloud_point`  
`simulation` 

## Usage
    `git clone git@github.com:GZHOUW/RSP_Project.git` to your workspace.  
    each package's usages are included in their `README` file   
    
## dependency
```
sudo apt-get install ros-melodic-kobuki-*
sudo apt-get install ros-melodic-ecl-streams
sudo apt-get install libusb-dev
sudo apt-get install libspnav-dev
sudo apt-get install ros-melodic-joystick-drivers
sudo apt-get install bluetooth
sudo apt-get install libbluetooth-dev
sudo apt-get install libcwiid-dev
sudo apt-get install ros-melodic-move-base*
sudo apt-get install ros-melodic-map-server*
sudo apt-get install ros-melodic-amcl*
sudo apt-get install ros-melodic-navigation*
sudo apt-get install ros-melodic-bfl
sudo apt-get install ros-melodic-openni2-launch
sudo apt-get install ros-melodic-yocs-velocity-smoother
```
## Result
Goals:
`Runnable model and test routine in gazebo`   
`Robot could locate itself in a well-known environment    `
`Robot could detect static obstacles`  

   Detecting human model with laser sensor is unrealistic in both gazebo and real-life(too many noise and parameters to set, we spent lots of time testing it and we gave up). We do managed to detect pillars and other objects, so we changed our goal to detect nearby obstacles in the area laser senser covers. All the delivery goals are achieved.
   
## Hardware
  
