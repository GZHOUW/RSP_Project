# RSP_Project 
## Guangwei_Zhou(gzhou11@jh.edu) 
## Jiahe_Xu(jxu109@jh.edu)

In this project, use `gazebo` for simulation, `move_base` package for planning. We also used clusting methods with cloudpoint data for object detection.   

In `follow_path` package we assigned a path and feed the path to `move_base`, then the turtlebot will follow the trajecctory if it is reachable.   

In `move_turtlebot` package we let turtlebot randomly move in one of the assigned directions according to `local_costmap` and laser sensor.  

In `turtlebot_planner` package we used RTT method and implemented a `move_base` plugin for planning.

Package `front_scan` and `object_detection` use laser sensor to detect obstacles and potential human figures with clusting methods(KD-tree)  

Package `cloud_point` is used to convert original laser data to point cloud data.  

In our tests, we used perfect maps and map generated by `gmapping`, in the demo (Readme files is each package), we only use perfect map for better result.


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

