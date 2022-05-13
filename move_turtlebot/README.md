# move_turtlebot

## Overview

This package is used to move a turlebot in random directions. It will use `local_costmap` to check the next steps. 

## Parameters
In `move_turtlebot`, we need to specify some parameters.
`step_length`:the distance we move in X or Y direction
`step_size`: how many steps we will take
`x` `y` `theta` ,initial position, if you change the value you should also change the position in corresponding `.yaml` file.


## Run Demo

1. Build the package
    
    `cd {workspace_dir}`
    
    `catkin build`
    
2. Run the launch file
    
    `roslaunch move_turtlebot move_turtlebot.launch`