# follow_path

## Overview

This package is used to move a turlebot in a give path. The path should be saved in `config/path.txt`.
You can specify the dir in `launch/follow_path.launch`. In the `path.txt` file, the first line contains a number, which is the number of goals. The following `n` lines contains n goal positions in `x,y,theta` format. If any point is unreachable, the node will stop. 


## Run Demo

1. Build the package
    
    `cd {workspace_dir}`
    
    `catkin build`
    
2. Run the launch file
    
    `roslaunch follow_path follow_path.launch`