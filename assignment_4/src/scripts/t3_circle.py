#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
PI = 3.1415926535897

def tbot_circle():
	rospy.init_node('circlebot', anonymous=True)
	pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
	#rate = rospy.Rate(10)
	vel = Twist()

	speed = 0.2
	angle = 360
	rad = float(angle*2*PI/360)

	while not rospy.is_shutdown():
		vel.linear.x = speed
		vel.linear.y = 0
		vel.linear.z = 0
		vel.angular.x = 0
		vel.angular.y = 0
		vel.angular.z = (speed * 2)

		t0 = rospy.Time.now().to_sec()
		cur_angle = 0

		while(cur_angle < rad):
			pub.publish(vel)
			t1 = rospy.Time.now().to_sec()
			cur_angle = speed * (t1-t0)

		vel.linear.x = 0
		vel.angular.z = 0
		pub.publish(vel)
		rospy.spin()


if __name__ == '__main__':
	try:
		#r = input("r = ")
		tbot_circle()
	except rospy.ROSInterruptException:
		pass
