#! /usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
 


def callback(msg):
    sweep = list(msg.ranges[0:360])                         # Make the list of laser scan data
    sweep = [v for v in sweep if not v > 5]                 # Filter out inf data
    left_sweep = sweep[10:90]                               # Took value from 10 degree to 90 degree for left sweep
    right_sweep = sweep[270:350]                            # Took value from 270 degree to 350 for left sweep 
    mean_right = sum(right_sweep)/len(right_sweep)       
    mean_left = sum(left_sweep)/len(left_sweep)
    error = mean_left - mean_right                           # Error is defied as average of left distances minus average of right distance

    velocity = 0.2                                           # Kept constant velocity

    kp = 0.2                                                 # Proportional coefficient

    if (error > 0.05 ):                                      # Condition for error condition
        velocity_msg.angular.z = kp * error 
        velocity_msg.linear.x = velocity

    elif (error < -0.05):
        velocity_msg.angular.z = kp * error 
        velocity_msg.linear.x = velocity

    else: 
        velocity_msg.angular.z = 0 
        velocity_msg.linear.x = velocity



rospy.init_node('wall_follow', anonymous=True)
velocity_msg=Twist()
pub = rospy.Publisher('cmd_vel', Twist, queue_size= 10)
scan_sub = rospy.Subscriber('scan', LaserScan,callback)
rate = rospy.Rate(10)
while not rospy.is_shutdown():
    pub.publish(velocity_msg)
    rate.sleep()
    pass