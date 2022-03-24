# Assignment 6: Line Following and Apriltag Tracking

This ```Readme``` contains information about what is in package  "assignment6_trackingandfollowing" and how to launch the files. Demo video of all task is in ```assignment6_trackingandfollowing/Videos```.

### Learning Outcomes

1. Manipulating image data for tracking a point.
2. Implementing path tracking controllers in Python

### Camera Calibration

When starting the assignment the Raspberrypi camera interface was setup and relevant packages were installed by following the instructions in the [TurtleBot3 manual](https://emanual.robotis.com/docs/en/platform/turtlebot3/appendix_raspi_cam/). For the calibartion we used the instructions for monocular camera given on the following [link](http://wiki.ros.org/camera_calibration)

The camera calibration was performed by running the following commands -
  - roscore
  - ssh into the TurtleBot and run the command roslaunch raspicam_node camerav2_1280x960.launch enable_raw:=true
  - In a new terminal run the command: rosrun camera_calibration cameracalibrator.py --size 8x6 --square 0.0245 image:=/raspicam_node/image camera:=/raspicam_node 
  - The packages for the camera calibration installed by following the instructions from [here](http://wiki.ros.org/camera_calibration).
  - The checkerboard used was a 8x6 board printed on an A4 sheet.
  
### Line Follower

This task had two parts. The one part was to follow the track in the gazebo environment and the second was to deploy it on the real turtlebot. In the scripts folder, you will find linefollower_sim.py and follow_line_step_hsv.py. The former py script is used for simulation and the later on the real turtlebot. The difference is in the values of Kp and Kd, and hsv values. For each of the scripts, you will also find a launch file in launch folder. Use turtlebot3_follow_line.launch to launch a gazebo simulation of turtlebot following a yellow line. Use turtlebot3_follow_line_real.launch to make a real turtlebot3 follow a red line. 

For simulation:
1) Start roscore.
2) launch the simulation launch file. (roslaunch assignment6_trackingandfollowing turtlebot3_follow_line.launch)
Output:

For real turtlebot:
1) Start roscore. (remote pc)
2) ssh into the turtlebot.
3) Bringup the camera.(roslaunch turtlebot3_bringup turtlebot3_rpicamera.launch) (turtlebot3)
(Note: You may also do the turtlebot3 bringup before the camera bringup ---> roslaunch turtlebot3_bringup turtlebot3_robot.launch)
4) Launch the line following node. (roslaunch assignment6_trackingandfollowing turtlenot3_follow_line_real.launch)

### AprilTag Follower

Download [apriltag_ros](https://github.com/AprilRobotics/apriltag_ros) and [apriltag](https://github.com/AprilRobotics/apriltag) repositories from github. Once you download these repositories, you may want to do catkin_make_isolated, since because of outside packages catkin_make will likely fail.
In the scripts folder there is py script called tagfollower.py that we used for tag following. Subsequent launch file for this is named - continuous_detection.launch.
To run this on a turtlebot, make sure you edit the tag family in the tags.yaml file in the config folder of apriltag_ros. (We have tag id 0 and 19).

Commands:
1) roscore (remote pc)
2) ssh into the turtlebot.
3) Bringup the camera.(roslaunch turtlebot3_bringup turtlebot3_rpicamera.launch) (turtlebot3)
4) rosrun image_transport republish compressed in:=camera/image raw out:=camera.image_raw (remote pc)
We had to do this to resolve the synchronization error that we were getting between camera/image and camera/camera_info.
5) Now launch the continuous detection node. (roslaunch assignment6_trackingandfollwoing continuous_detection.launch) (remote pc)
6) Launch rqt image viewer. (rqt_image_view (remote pc))



