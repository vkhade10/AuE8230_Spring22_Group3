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

![5203](https://user-images.githubusercontent.com/79803663/162640408-c0084081-2134-45f2-afc9-5523418cd303.png)

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

![krato_5203](https://user-images.githubusercontent.com/79803663/162640416-8e039f6d-f20e-468a-85bc-a3bcbab16e22.png)

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

The video 'LDS_slam.mkv' shows the map generation process using LDS lidar.

### 2.Run the SLAM node and save the map using the Hokuyo 3D lidar.

The map saved using the Hokuyo lidar is shown below,

<img src="https://github.com/Abetelgeusian/AuE8230_Spring22_Group3/blob/master/assignment7/Map%20files/hokuyo_map.png" width="50%" height="50%">

The video 'Hokuyo_slam.mkv' shows the map generation process using LDS lidar.

## Observations & Issues

I) Issues

Upon using the provided urdf & message files for the hokuyo lidar to update the turtlebot3's configuration, several issues were encountered..

- With the initially given parameter values, '/scan' topic didnot output any data and an error 'Laser is mounted upwards' was thrown.
- On reducing the resolution and the min & max angle values, we were able to receive '/scan' topic information. The updated values were,

```
<resolution>0.025</resolution>
<min_angle>0.0</min_angle>
<max_angle>0.1</max_angle>
```

- But even with this update, the resultant map was coming out to be distorted(shown below).

<img src="https://github.com/Abetelgeusian/AuE8230_Spring22_Group3/blob/master/assignment7/videos/Hokuyo_slamdistorted.gif" width="50%" height="50%">


- To fix this, further modifications were made to the gazebo description. The no. of samples were both made equal to 74 (initially tried with 100 but no improvements were obtained). Further, the sensor type was changed to ```ray``` from ```gpu_ray```

Thus the final description of Hokuyo lidar in the turtlebot3's gazebo description was,

```
<gazebo reference="base_scan">
    <sensor type="ray" name="head_hokuyo_sensor">
      <pose>0 0 0 0 0 0</pose>
      <visualize>true</visualize>
      <update_rate>10</update_rate>
      <ray>
        <scan>
          <horizontal>
            <samples>74</samples>
            <resolution>1.0</resolution>
            <min_angle>-1.83</min_angle>
            <max_angle>1.83</max_angle>
          </horizontal>
          <vertical>
            <samples>74</samples>
            <resolution>0.025</resolution>
            <min_angle>0.0</min_angle>
            <max_angle>0.1</max_angle>
          </vertical>
        </scan>
        <range>
          <min>0.10</min>
          <max>30.0</max>
          <resolution>0.01</resolution>
        </range>
        <noise>
          <type>gaussian</type>
          <!-- Noise parameters based on published spec for Hokuyo laser
               achieving "+-30mm" accuracy at range < 10m.  A mean of 0.0m and
               stddev of 0.01m will put 99.7% of samples within 0.03m of the true
               reading. -->
          <mean>0.0</mean>
          <stddev>0.01</stddev>
        </noise>
      </ray>
      <plugin name="gazebo_ros_head_hokuyo_controller" filename="libgazebo_ros_laser.so">
        <topicName>scan</topicName>
        <frameName>base_scan</frameName>
      </plugin>
    </sensor>
  </gazebo>
```
  
  
II) Observations
- The LDS lidar seems to take lesser time to localize & locate the bot in the environment. This could be due to the fact that LDS lidar only uses horizontal scan values whereas in case of Hokuyo, both vertical & horizontal scan values are used
- Using Hokuyo lidar seems to result in more accurate mapping and the navigation path being more optimal (see following figures).

Navigation with Hokuyo:
<img src="https://github.com/Abetelgeusian/AuE8230_Spring22_Group3/blob/master/assignment7/videos/Hokuyo_Nav.gif" width="50%" height="50%">   

Navigation with LDS:

<img src="https://github.com/Abetelgeusian/AuE8230_Spring22_Group3/blob/master/assignment7/videos/LDS_Nav.gif" width="50%" height="50%">  
