#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from apriltag_ros.msg import AprilTagDetectionArray
import numpy as np


angle = 0
linear_dist = 0

class Apriltag_follower(object):

    def __init__(self,pub):
        self.bridge = CvBridge()
        #to use the CvBridge, you will need to transfer the message from compressed to raw (follow the instructions)
        self.pub = pub
        self.image_sub = rospy.Subscriber('/tag_detections_image', Image, self.camera_callback)
        self.sub = rospy.Subscriber('/tag_detections', AprilTagDetectionArray,self.callback)



    def camera_callback(self, data):
        cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")
        cv2.imshow("Scan", cv_image)
        cv2.waitKey(1)

    def callback(self, data):
        #rospy.loginfo("I'm Here ! ")
        global angle, linear_dist
        
        try:
            vel_msg = Twist()
            angle = data.detections[0].pose.pose.pose.position.x        #compensate the width to determine the right x difference
            linear_dist = data.detections[0].pose.pose.pose.position.z           #the z position represents the depth from the camera cooridnates
            if linear_dist > 0.05:
                vel_msg.linear.x = linear_dist
                vel_msg.angular.z = -angle * 10 
                self.pub.publish(vel_msg)
            elif linear_dist < 0.035:
                vel_msg.linear.x = -((0.05-linear_dist)/0.05) * 0.1
                vel_msg.angular.z = angle * 10 
                self.pub.publish(vel_msg)
            else:                
                vel_msg.linear.x = 0
                vel_msg.angular.z = 0
                self.pub.publish(vel_msg)
        except IndexError:
                vel_msg.angular.z = -0.1
                self.pub.publish(vel_msg)
               # rospy.loginfo('Tag not detected')


def main():
    try:
        rospy.init_node('april_tag_node', anonymous=True)
        pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        Apriltag_follower(pub)
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    # rate = rospy.Rate(10)
    # while not rospy.is_shutdown():
    #     rate.sleep()

if __name__ == '__main__':
    while True:
        main()