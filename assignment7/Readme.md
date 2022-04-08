# Assignment 7: SLAM

This ```Readme``` contains information about what is in package  "assignment7" and how to launch the files. Demo video of all task is in ```assignment7/Videos```.

### Learning Outcomes

1. SLAM on the real turtlebot
2. Comparison between two different lidars for building the map for SLAM in gazebo

## Part 1: SLAM and Navigation

This part done completely on the physical turtlebot.

### 1. Run gmapping SLAM node and save the map.

Run the folllowing commands for Gmapping

``` roslaunch turtlebot3_bringup turtlebot3_robot.launch```  (On Turtlebot)

```roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch``` -  For Teleoperation

```roslaunch turtlebot3_slam turtlebot3_slam.launch``` (On Remote PC)

For saving the map -

```rosrun map_server map_saver -f ~/map``` 

Here ```map``` is the name of map file.

Run the Navigation -

```roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=$HOME/map.yaml```

Specifying the name of map in place of ```map.yaml```

Specify the initial pose by - 

1. Clicking the 2D Pose Estimate button in the RViz menu
2. Clicking the 2D Nav Goal button in the RViz menu.

### 2. Run karto SLAM node and save the map

i) Install dependent packages on PC.

```sudo apt install ros-noetic-slam-karto```

ii) Launch the Karto SLAM node.

```roslaunch turtlebot3_slam turtlebot3_slam.launch slam_methods:=karto```

Run the folllowing commands for Gmapping

``` roslaunch turtlebot3_bringup turtlebot3_robot.launch```  (On Turtlebot)

```roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch``` -  For Teleoperation

```roslaunch turtlebot3_slam turtlebot3_slam.launch slam_methods:=karto``` (On Remote PC)

For saving the map -

```rosrun map_server map_saver -f ~/map``` 

Here ```map``` is the name of map file.


Run the Navigation -

```roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=$HOME/map.yaml```

Specifying the name of map in place of ```map.yaml```

Specify the initial pose by - 

1. Clicking the 2D Pose Estimate button in the RViz menu
2. Clicking the 2D Nav Goal button in the RViz menu.

## Part 2: Comparison between two lidars

This part done in Gazebo simulation environment

### 1. Run the SLAM node and save the map using the lidar that is already present on the turtlebot

The map saved using the already present LDS lidar is shown below,
<img src="https://github.com/Abetelgeusian/AuE8230_Spring22_Group3/blob/master/assignment7/Map%20files/map.png" width="50%" height="50%">

### 2.Run the SLAM node and save the map using the Hokuyo 3D lidar.

The map saved using the already present Hokuyo lidar is shown below,
<img src="https://github.com/Abetelgeusian/AuE8230_Spring22_Group3/blob/master/assignment7/Map%20files/hokuyo_map.png" width="50%" height="50%">

