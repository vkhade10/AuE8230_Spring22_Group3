#! /usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

def callback(msg):
    sweep = list(msg.ranges[0:359])
    
    front1 = sweep[0:40]
    front2 = sweep[320:359]
    front_dist = front1 + front2
    front_dist = [i for i in front_dist if not i==0] # taking out the zero values
    front_dist = min(front_dist)

    thr_dist = 0.5

    left_sect = sweep[40:90]
    left_sect = min(left_sect) # /len(left_sect)

    right_sect = sweep[270:319]
    right_sect = min(right_sect) #/ len(right_sect)
    
    error = left_sect - right_sect

    vel = 0.5
    
    if (front_dist < thr_dist): # when front dist is less than the threshhold then you turn otherwise go
        if error < 0: # when right sect is higher
            vel_msg.linear.x = -0.1
            vel_msg.angular.z = 1.5 * error
        else:
            vel_msg.linear.x = -0.1
            vel_msg.angular.z = 1.5 * error
    else:
        
        vel_msg.linear.x = vel
        vel_msg.angular.z = 0
        

rospy.init_node('wander', anonymous=True)
vel_msg=Twist()
pub = rospy.Publisher('cmd_vel', Twist, queue_size= 10)
sub = rospy.Subscriber('scan', LaserScan,callback)
while not rospy.is_shutdown():
    pub.publish(vel_msg)
    pass
rospy.spin()