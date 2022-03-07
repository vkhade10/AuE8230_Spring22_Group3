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


## PART 2: Obstacle Avoidance

Created a python code wander.py where we implemented obstacle avoidance using scan data. 2D Lidar used for scan data.There is not goal location, so the robot simple wander around the room avoiding obstacles.


Launch File - ```turtlebot3_obstacle.launch```

World File - ```turtlebot3_obstacles.world```

Script File - ```wander.py```

This command ```roslaunch assignment5_wallfollowingandobstacleavoidance turtlebot3_obstacle.launch``` launches the world and sript file.


![Screenshot from 2022-03-07 02-34-34](https://user-images.githubusercontent.com/79803663/156989651-a1b29576-60ca-4f1e-a2b5-f6e6c11c02e4.png)


## PART 3: Obstacle Avoidance in Real World

Implemented the obstacle avoidance code on the actual Turtlebot. Create a makeshift obstacle course with boxes /bags/everyday stuff and demonstrated working of code. 


Launch File - ```wander_real.launch```

Script File - ```wander.py```

This command ```roslaunch assignment5_wallfollowingandobstacleavoidance wander_real.launch``` launches the sript file. 



![20220307_002842_1](https://user-images.githubusercontent.com/79803663/156991090-8146e207-9f30-42ac-92a7-4abd728b2050.gif)
