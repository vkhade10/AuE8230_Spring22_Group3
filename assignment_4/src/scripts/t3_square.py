#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
PI = 3.1415926535897

def square_olp():
	print("Let's make a square.")
	rospy.init_node('bot_square', anonymous=True)
	pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
	#rate = rospy.Rate(10)
	vel = Twist()


	# Initial set values
	lvel = 0.1  # linear velocity
	avel = 0.25  # angular velocity
	d = 0.46       # dimension of square
	angle = (90*2*PI)/360  # rotation angle

	#if (isforw == 'True'):
		#vel.linear.x = abs(speed)
	#else:
		#vel.linear.x = -abs(speed)

	# Initial values
	vel.linear.x = 0
	vel.linear.y = 0
	vel.linear.z = 0
	vel.angular.x = 0
	vel.angular.y = 0
	vel.angular.z = 0

	while not rospy.is_shutdown():
		# Loop for 4 sides of square
		for i in range(4):

			# Straight in line
			t0 = rospy.Time.now().to_sec()
			cur_dis = 0
			vel.linear.x = lvel

			while(cur_dis < d):
				pub.publish(vel)
				#rospy.loginfo(vel.linear.x)

				t1 = rospy.Time.now().to_sec()
				cur_dis = lvel * (t1-t0)

			vel.linear.x = 0
			pub.publish(vel)

			# Rotation
			vel.angular.z = avel
			t0 = rospy.Time.now().to_sec()
			cur_angle = 0

			while(cur_angle < angle):
				pub.publish(vel)
				#rospy.loginfo(vel.angular.z)
				t1 = rospy.Time.now().to_sec()
				cur_angle = avel * (t1-t0)
			vel.angular.z = 0
			pub.publish(vel)
		rospy.spin()


if __name__ == '__main__':
	try:
		#r = input("r = ")
		square_olp()

	except rospy.ROSInterruptException:
		pass
