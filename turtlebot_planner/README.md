# Turtlebot Planner

## Overview

This package is a planner for turtlebot2. The planner uses the RRT algorithm, one of the most widely used sampling-based planning algorithms. The planner inherits the `nav_core::BaseGlobalPlanner` class so that this planner is used when a command is sent, instead of the original global planner.

## Environment

- Ubuntu 16.04 LTS
- ROS Kinetic
- Gazebo
- catkin

## Dependencies

### Catkin Dependencies

- roscpp
- pluginlib
- nav_core
- costmap_2d
- base_local_planner

### External Dependencies

- turtlebot_gazebo
    
    Install with command: `sudo apt-get install ros-kinetic turtlebot-gazebo`
    
- turtlebot_navigation
    
    Install with command: `sudo apt-get install ros-kinetic turtlebot-navigation`
    

## Run Demo

1. Build the package
    
    `cd {workspace_dir}`
    
    `catkin build`
    
2. Run the launch file
    
    `roslaunch turtlebot_planner turtlebot_planner.launch`
    
    You should see rviz and gazebo gui initialized. Logs will be displayed in terminal.
    
    Wait a few seconds for all the nodes to start up. Do not input commands until you see “odom received” in terminal. 
    
3. In rviz, use “2D Nav Goal” to input a goal pose for the robot to move to. You may see error messages in terminal, but they do not affect the process of the program. The robot model in simulation (both rviz and gazebo) will start to move towards the goal pose.
<img src="short_demo.gif" width = "800"/>
