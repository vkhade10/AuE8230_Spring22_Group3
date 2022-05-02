
# AuE8230 Final Project - Turtlebot3 Autonomous Maneuvers (Group 3)

This is ROS-based project using Turtlebot3 Burger in simulation (Gazebo) and real-world environmnet. Final project as part of class AuE8930 - Autonomy Science and Systems at CUICAR.


# Project Overview:

Project divided into autonomous maneuvers in simulation and real world environment.

## The Environment

### Gazebo Environment

![world_1](https://user-images.githubusercontent.com/79803663/166186834-9a88838c-8bbf-4044-84e5-234fb0fb9f52.png)


### Real World Environment

## Tasks

The robot will have to complete the following tasks:

### Task 1: Wall following/Obstacle avoidance - 
It must successfully follow the wall and avoid the obstacles until it reaches the yellow line. Create a map of this corridor using a SLAM package of choice.
### Task 2: Line following 
The Turtlebot must successfully follow the yellow line. 
### Task 3:  Stop Sign Detection -
While navigating the yellow line, the the Turtlebot should stop at the stop sign for 3 seconds before continuing. The stop-sign will be detected by Tiny YOLO.
### Task 4: April Tag tracking - 
For this task you will need to spawn another TB3 in the environment in the empty space past the yellow line and attach an AprilTag to the robot. The Turtlebot3 with the AprilTag  will be teleoperated by the user and the preceding TB3 needs to track its motion.

# Results:

## Gazebo Simulations

https://user-images.githubusercontent.com/79803663/166188070-82cc406d-f6ff-49c6-94ee-ab7811f7afe2.mp4


## Real-world Demo 


## Running the repo

The project was created on ROS1 (noetic) on Ubuntu 20.04.

Use the below lines in your terminal to start the launch files.

1. For Gazebo Simulation -

 - `roslaunch aue_finals turtlebot3_autonomy_finals.launch`

2. For Real World -
  - 1. Bringup Turtlebot and Raspicam -
       - `roslaunch turtlebot3_bringup turtlebot3_robot.launch`
       - `roslaunch turtlebot3_bringup turtlebot3_rpicamera.launch`
  - 2. Integrated Launch File


### Dependencies

1. [Turtlebot3 packages](https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/)
2. [Turtlebot3 simulation](https://github.com/ROBOTIS-GIT/turtlebot3_simulations)
3. [ROS-perception Open CV](https://github.com/ros-perception/vision_opencv)
4. [Apriltag ROS](https://github.com/AprilRobotics/apriltag_ros)
5. [Darknet ROS (YOLO)](https://github.com/leggedrobotics/darknet_ros)
