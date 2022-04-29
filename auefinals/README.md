
## AuE8230 Final Project Gazebo Model - Group 3

Dependencies that are not included:

* The TB3 packages

To bring up the Gazebo Model:

roslaunch aue_finals turtlebot3_autonomy_finals.launch


There are 3 sections to this project:

* Task 1: Wall following/Obstacle avoidance - The Turtlebot starts here. It must successfully follow the wall and avoid the obstacles until it reaches the yellow line.
* Task 2:
- Line following - The Turtlebot must successfully follow the yellow line.
- Stop Sign detection - While navigating the yellow line, the the Turtlebot should stop at the stop sign for 3 seconds before continuing. The stop-sign will be detected by TinyYOLO.
* Task 3: AprilTag tracking - Spawn another TB3 in the environment past the line. Attach an AprilTag onto the robot. You must teleop this TB3 (there are several packages available online, to enable teleop terminals; use the namespace concept to send seperate /cmd_vel values to seperate robots), and the preceding TB3 should follow it.


