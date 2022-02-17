This package "assignment3_turtlebot3" contains information about what is in package and how to launch the files.

Scripts

In the scripts folder there are three python scripts: circle, square and wall

circle.py script uses cmd_vel topic and moves the turtlebot in circular manner. User can change the values for the linear and angular velocities in the script. Too high values of angular and linear velocities will make the turtlebot3 unstable.

square.py is openloop method to make the turtlebot trace a square. Since this is openloop method, the square drawn will not be a perfect square. Depending on angular and linear velocity, it may draw something else than a sqaure. 

wall.py script is used to stop the turtlebot3 once it detects a wall. It uses a subscriber to the topic "scan" and publishes on topic "cmd_vel". This particular task asked us to scan for the central values and stop. Stopping distance can be defined in the script.

LaunchFiles

There are two launch files.
move.launch -  this files is used to load an empty world in gazebo and then execute either circle or square. User can choose which file (circle or square) to run.

turtlebot3_wall.launch - this launch files launches a world with wall at a certain distance. Then exceutes the wall.py file.

commands:
Task 1:
for circle in empty world: 
roslaunch assignment3_turtlebot3 move.launch code:=circle

for square in empty world: 
roslaunch assignment3_turtlebot3 move.launch code:=square

Task 2: Emergency braking
roslaunch assignment3_turtlebot3 turtlebot3_wall.launch
