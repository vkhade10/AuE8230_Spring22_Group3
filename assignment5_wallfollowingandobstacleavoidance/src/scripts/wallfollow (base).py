#! /usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

def callback(msg):
    sweep = list(msg.ranges[0:360])
    sweep = [i for i in sweep if not i>5]
    front1 = sweep[20:80]
    front2 = sweep[351:359]
    front_dist = front1 + front2
    front_dist = sum(front_dist) / len(front_dist)

    thr_dist = 1.5

    left_sect = sweep[10:70]
    left_sect = sum(left_sect) / len(left_sect)

    right_sect = sweep[280:350]
    right_sect = sum(right_sect) / len(right_sect)
    
    error = left_sect - right_sect

    vel = 0.5
    e11,e21,e31 = 0,0,0
    u = 0
    kp = 0.4
    ki = 0
    kd = 0.25
    k11 = kp + ki + kd
    k21 = -kp - 2*kd
    k31 = kd

    if (front_dist > thr_dist):
        print("front dist is ")
        print(front_dist)
        vel_msg.linear.x = vel
        e31 = e21
        e21 = e11
        e11 = error
        u = u + k11*e11 + k21*e21 + k31*e31
        vel_msg.angular.z = u
    else:
        print("front dist is ")
        print(front_dist)
        print("moving backwards")
        vel_msg.linear.x = vel - 0.35
        vel_msg.angular.z = 1.2 * error
        

rospy.init_node('wall_follow', anonymous=True)
vel_msg=Twist()
pub = rospy.Publisher('cmd_vel', Twist, queue_size= 10)
sub = rospy.Subscriber('scan', LaserScan,callback)
rate = rospy.Rate(10)
while not rospy.is_shutdown():
    pub.publish(vel_msg)
    rate.sleep()
    pass