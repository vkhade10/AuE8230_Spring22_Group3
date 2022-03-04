# Assignment 5: Turtlebot3 Simulation

This ```Readme``` contains information about what is in package  "assignment5_wallfollowingandobstacleavoidance" and how to launch the files. Demo video of all task is in ```assignment5_wallfollowingandobstacleavoidance/Videos```.

Outcomes:
1. Manipulating scan data for navigation
2. Implementing P or PD controllers in Python

## PART 1: Wall Following

In the first part of this assignment we create a python code ```wall_follower.py``` that extracts
the ```/scan data``` and chooses data points that reflect the sides of the robot. Feed the robot a
constant forward velocity and attempt to avoid the wall. We write a simple P controller to
maintain the equal distances from each face of the wall. 

Launch File - ```turtlebot3_wallfollowing.launch```

World File - ```turtlebot3_wallfollowing.world```

Script File - ```wall_follower.py```

This command ```roslaunch assignment5_wallfollowingandobstacleavoidance turtlebot3_wallfollowing.launch``` launches the world and sript file.


![wall_follow](https://user-images.githubusercontent.com/79803663/156851871-75ac4c50-8e53-487e-8c26-ca57bb2c3d19.png)
