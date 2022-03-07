#!/usr/bin/env python

import rospy 
import numpy as np
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


def callback(msg):
    sweep = list(msg.ranges[0:359])                   # Sweep Data in list data format
    #print('sweep',(sweep))
    front_sector = sweep[0:20] + sweep[-20:]
    #print('Front_sector',(front_sector))
    front_sector = [v for v in front_sector if not v > 4]        # Filter the invalid data like inf
    front_sector = [v for v in front_sector if not v ==0]
    left_sector  = sweep[20:90]                                   # Left sectoring from 20 degree to 90 degree
    #print('lefffffff',len(left_sector))
    left_sector = [v for v in left_sector if not v > 4]
    left_sector = [v for v in left_sector if not v ==0]
    right_sector = sweep[-90:-20]                                 # Right sectoring from 20 degree to 90 degree
    #print('rigghtttttttt',(right_sector))
    right_sector = [v for v in right_sector if not v > 4]
    right_sector = [v for v in right_sector if not v ==0]

    # Average of data from all sectors
    avg_front = sum(front_sector)/len(front_sector)
    avg_right = sum(right_sector)/len(left_sector)
    avg_left = sum(left_sector)/len(front_sector)

    print('Front',(avg_front))
    print('Left',(avg_left))
    print('Right',(avg_right))

    # Condition if obstacle in front
    if avg_front < 1:
        if avg_left>avg_front:                        # Left turn condition
            velocity_msg.angular.z = 1.2
            velocity_msg.linear.x = 0.08
        elif avg_right>avg_front:                     # Right turn condition
            velocity_msg.angular.z = -1.2
            velocity_msg.linear.x = 0.08  
        else:                                         # condition if obstale is in front, right and left
            velocity_msg.angular.z = 1.2
            velocity_msg.linear.x = 0 


    else:                                            # Condition when no obstacle
        velocity_msg.angular.z = 0
        velocity_msg.linear.x = 0.15
    
    pub.publish(velocity_msg)


rospy.init_node('wander', anonymous=True)
velocity_msg=Twist()
pub = rospy.Publisher('cmd_vel', Twist, queue_size= 10)
scan = rospy.Subscriber('scan', LaserScan,callback)
rate = rospy.Rate(10)
while not rospy.is_shutdown():
    pub.publish(velocity_msg)
    rate.sleep()
    pass

