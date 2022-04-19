#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


e11 = 0
e21 = 0
e31 = 0
u = 0

def callback(msg):
    sweep = list(msg.ranges[0:359]) # lidar reading
    sweep = [i for i in sweep if not i>1] # ignoring values above 4 m
    front1 = sweep[0:20]
    front2 = sweep[-20:]
    front_dist = front1 + front2
    front_dist = sum(front_dist) / len(front_dist) # taking average of 20 degree lidar sweep

    thr_dist = 1.5 # threshold distance



# following lines and section is used for declaring discretised PD controller
    vel = 0.15 # velocity value

    global e11
    global e21
    global e31
    global u
    #rospy.loginfo(" e11 %.2f ", e11)
    kp = 0.4
    ki = 0.0
    kd = 0.2
    k11 = kp + ki + kd
    k21 = -kp - 2*kd
    k31 = kd
    error = 0.5

    if (front_dist < 1.5 and front_dist > 0.25): # if the front_dist is less than the thresh hold distance,
        # print("front dist is ")
        # print(front_dist)

        left_sect = sweep[11:70]
        left_sect = sum(left_sect) / len(left_sect) # average of the left sector

        right_sect = sweep[-82:-12]
        right_sect = sum(right_sect) / len(right_sect) # average of the right sector

        error = left_sect - right_sect # defining error term

        vel_msg.linear.x = vel
        e31 = e21
        e21 = e11
        e11 = error
        u = u + k11*e11 + k21*e21 + k31*e31
        vel_msg.angular.z = u

    elif (front_dist < 0.25): # when front dist is less than the threshhold then you turn otherwise go

        left_sect = sweep[20:100]
        left_sect = min(left_sect) # /len(left_sect)

        right_sect = sweep[260:320]
        right_sect = min(right_sect) #/ len(right_sect)

        error = left_sect - right_sect

        if error < 0: # when right sect is higher
            vel_msg.linear.x = 0.2
            vel_msg.angular.z = 3 * error
        else:
            vel_msg.linear.x = -0.1
            vel_msg.angular.z = 1.5 * error
    else:
        # print("front dist is ")
        # print(front_dist)
        # print("moving backwards")
        vel_msg.linear.x = vel
        vel_msg.angular.z = 1.5 * error
    rospy.loginfo(front_dist)
    rospy.loginfo(error)




rospy.init_node('wall_follow_avoidance', anonymous=True)
vel_msg=Twist()
pub = rospy.Publisher('cmd_vel', Twist, queue_size= 10)
sub = rospy.Subscriber('scan', LaserScan,callback)
while not rospy.is_shutdown():
    pub.publish(vel_msg)
    pass
rospy.spin()
