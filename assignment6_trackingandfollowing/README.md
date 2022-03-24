# Assignment 6: Line Following and Apriltag Tracking

This ```Readme``` contains information about what is in package  "assignment6_trackingandfollowing" and how to launch the files. Demo video of all task is in ```assignment6_trackingandfollowing/Videos```.

### Learning Outcomes

1. Manipulating image data for tracking a point.
2. Implementing path tracking controllers in Python

## Camera Calibration

When starting the assignment the Raspberrypi camera interface was setup and relevant packages were installed by following the instructions in the [TurtleBot3 manual](https://emanual.robotis.com/docs/en/platform/turtlebot3/appendix_raspi_cam/).

The camera calibration was performed by running the following commands -
  - roscore
  - ssh into the TurtleBot and run the command roslaunch raspicam_node camerav2_1280x960.launch enable_raw:=true
  - In a new terminal run the command: rosrun camera_calibration cameracalibrator.py --size 8x6 --square 0.0245 image:=/raspicam_node/image camera:=/raspicam_node 
  - The packages for the camera calibration installed by following the instructions from [here](http://wiki.ros.org/camera_calibration).
  - The checkerboard used was a 8x6 board printed on an A4 sheet.
