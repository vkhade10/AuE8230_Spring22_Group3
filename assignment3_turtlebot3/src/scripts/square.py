#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
import math
Pie = math.pi

def sq_openl():
  # Starts a new node
  print("Let's make a square.")
  rospy.init_node('square_maker', anonymous=True)
  velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
  vel_msg = Twist()
  #Receiveing the user's input
  speed = 0.3 #float(input("Input your speed:"))
  ang_speed = 0.3 #float(input("Input your angular speed:"))
  side = 1 #float(input("Type side length:"))
  #isForward = input("Foward?: ")#True or False
  #rot = 0
  while not rospy.is_shutdown(): # uncomment this line to run in infinite loop.
  # to run in infinite loop, commen everything wil rot.
  #while rot < 4:
    move(side,speed,vel_msg,velocity_publisher)
    turn(ang_speed,vel_msg,velocity_publisher)
    #rot += 1
  rospy.spin()
    
def move(side,speed,vel_msg,velocity_publisher):
  vel_msg.linear.x = speed
  # we will need time to calculate distance covered.
  t1 = rospy.Time.now().to_sec()
  dist = 0
  
  while dist < side:
    velocity_publisher.publish(vel_msg)
    rospy.loginfo(vel_msg.linear.x)
    # we measure time at this point, to calculate distance.
    t2 = rospy.Time.now().to_sec()
    dist = speed * (t2-t1)     
  #After the loop, stops the robot
  vel_msg.linear.x = 0
  #Force the robot to stop
  velocity_publisher.publish(vel_msg)
    
def turn(ang_speed,vel_msg,velocity_publisher):
  vel_msg.angular.z = ang_speed
  t1 = rospy.Time.now().to_sec()
  ang = 0
     
  while ang < (Pie/2):
    velocity_publisher.publish(vel_msg)
    rospy.loginfo(vel_msg.angular.z)
    t2 = rospy.Time.now().to_sec()
    ang = ang_speed*(t2-t1)
  #After the loop, stops the robot
  vel_msg.angular.z = 0
  #Force the robot to stop
  velocity_publisher.publish(vel_msg)   
       

if __name__ == '__main__':
    try:
        #Testing our function
        sq_openl()
    except rospy.ROSInterruptException: pass
