#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

def embrake(msg):
	if msg.ranges[0] > 0.25:
		move.linear.x = 0.1
		move.angular.z = 0.0

	elif msg.ranges[0] < 0.25:
		move.linear.x = 0.0
		move.angular.z = 0.0

	pub.publish(move)

rospy.init_node('emergency_brake', anonymous=True)
sub = rospy.Subscriber('/scan', LaserScan, embrake)
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
move = Twist()

rospy.spin()
