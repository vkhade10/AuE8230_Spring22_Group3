#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist

def Kreis():
	#define node
	rospy.init_node('for_circle', anonymous=True)
	publisher = rospy.Publisher('/cmd_vel',Twist,queue_size = 10)
	vel_msg = Twist()
	rate = rospy.Rate(10) # this line keeps the while loop running at exact 10 hertz.
	
	print("Gandalf has nothing on me. I will pass.")
	radius = 0.1 #int(input("Enter desired radius: "))
	speed = 0.1 #int(input("Enter the angular velocity: "))
	
	while not rospy.is_shutdown():
		vel_msg.linear.x = radius
		vel_msg.linear.y = 0
		vel_msg.linear.z = 0
		vel_msg.angular.x = 0
		vel_msg.angular.y = 0
		vel_msg.angular.z = speed
		rospy.loginfo("Radius %f: ", radius) # Displays radius and coordinates of turtle.
		publisher.publish(vel_msg)
		rate.sleep()
		
if __name__ == '__main__':
	#rospy.wait_for_service('reset')
	#reset = rospy.ServiceProxy('reset', Kreis) 
	#reset()
	try:
		Kreis()
	except rospy.ROSInterruptException:
		rospy.loginfo("Node terminated")
		
