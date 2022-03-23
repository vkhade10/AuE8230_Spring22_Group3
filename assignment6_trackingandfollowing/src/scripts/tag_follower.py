#! /usr/bin/env python3

import rospy
from geometry_msgs.msg  import Twist
from apriltag_ros.msg import AprilTagDetectionArray


rospy.init_node('wtf_is_this', anonymous=True)
pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
move = Twist()
# rate = rospy.Rate(10)


def callback(msg):
	try:
		x = msg.detections[0].pose.pose.pose.position.x
		z = msg.detections[0].pose.pose.pose.position.z
		move.linear.x = z*10
		move.angular.z = -x*70
		pub.publish(move)
	except IndexError:
		x = 0.1
		z = 0
		move.linear.x = z*10
		move.angular.z = x
		pub.publish(move)
          
subscriber = rospy.Subscriber('/tag_detections', AprilTagDetectionArray, callback)
rospy.spin()
