#!/usr/bin/env python

import rospy
import time
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from darknet_ros_msgs.msg import BoundingBoxes
import numpy as np
import cv2

global d, det_flag
right_min = 0.3
left_min = 0.3
front_min = 10
d = 0
det_flag = 0
flag=1
e11 = 0
e21 = 0
e31 = 0
u = 0
kp = 0.45
ki = 0.003
kd = 0.1
k11 = kp + ki + kd
k21 = -kp - 2*kd
k31 = kd
num = 0
controller = 0
ind = 0
right_cone1 = []
front_cone1 = []
right_min1 = 0 
left_dist = 0
right_dist = 0 
left_theta = 0
right_theta = 0
front1 = 0
left_ar = 0
right_ar = 0
n_w = 0

class LineFollower(object):
    def __init__(self):
        self.bridge_object = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/image",Image,self.camera_callback)
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.callback_wall)
        self.stop_sign_subscriber = rospy.Subscriber('/darknet_ros/bounding_boxes' , BoundingBoxes, self.stop_sign_callback)
        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 10)
        self.rate = rospy.Rate(150)



    def stop_sign_callback(self,msg):
        global num
        if msg.bounding_boxes[len(msg.bounding_boxes)- 1].id == 11:
            prediction = msg.bounding_boxes
            for box in prediction:
                identified_class=box.Class
                probability = float(box.probability)
                area = abs(box.xmax-box.xmin)*abs(box.ymax-box.ymin)
                print(area,'area')
                print('probability',probability)
                if ((probability >= 0.4) and (area >=1000)): 
                    print('stop_sign_detected')
                    num = 1

    def camera_callback(self, data):
        global controller
        # We select bgr8 because its the OpneCV encoding by default
        cv_image = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8")

        # We get image dimensions and crop the parts of the image we dont need
        height, width, channels = cv_image.shape
        # print(f'height:{height}, width:{width}')
        # crop_img = cv_image[int((height/2)+100):int((height/2)+120)][1:int(width)] # original
        # crop_img = cv_image[1:int(height)][1:int(width)] #works
        crop_img = cv_image[460:480][1:640]

        # Convert from RGB to HSV
        hsv = cv2.cvtColor(crop_img, cv2.COLOR_BGR2HSV)

        # Define the Yellow Colour in HSV

        """
        To know which color to track in HSV use ColorZilla to get the color registered by the camera in BGR and convert to HSV. 
        """

        # Threshold the HSV image to get only yellow colors
        lower_yellow = np.array([20,100,100]) # 160 50 70
        upper_yellow = np.array([50,255,255])
        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

        # Calculate centroid of the blob of binary image using ImageMoments
        m = cv2.moments(mask, False)
   
        try:
            cx, cy = m['m10']/m['m00'], m['m01']/m['m00']
        except ZeroDivisionError:
            cx, cy = height/2, width/2

        if cx !=240 and cy !=320:
            controller = 1

        # Draw the centroid in the resultut image
        # cv2.circle(img, center, radius, color[, thickness[, lineType[, shift]]]) 
        cv2.circle(mask,(int(cx), int(cy)), 10,(0,0,255),-1)
        cv2.imshow("Original", cv_image)
        cv2.imshow("MASK", mask)
        cv2.waitKey(1)

        #################################
        ###   ENTER CONTROLLER HERE   ###
        #################################
        
        
        global u 
        global e11
        global e21
        global e31
        global flag, det_flag
        twist_object = Twist()
        e = cx - height/2 - 3
        # rospy.loginfo(str(e))
        e31 = e21
        e21 = e11
        e11 = e
        u = u + k11*e11 + k21*e21 + k31*e31
        twist_object.linear.x = 0.1
        twist_object.angular.z =  -u/390 # we use negative sign here, to turn away from the blob.

        if flag==1 and controller ==1 :

            # rospy.loginfo('K11:'+ str(k11))
            # rospy.loginfo('K21:'+ str(k21))
            # rospy.loginfo('K31:'+ str(k31))
            # rospy.loginfo('U:'+ str(u))
            print('line_controller',controller)
            #Otherwise turtlebot will go out of the track.
            print(twist_object.linear.x)
            print(twist_object.angular.z)
            # rospy.loginfo("ANGULAR VALUE SENT===>"+str(twist_object.angular.z))
            # Make it start turning
            # self.moveTurtlebot3_object.move_robot(twist_object)
            self.vel_pub.publish(twist_object)
            print('chaltay')
            det_flag =1

            global num
            global loop_once
            if num ==1:
                twist_object.linear.x = 0
                twist_object.angular.z = 0
                self.vel_pub.publish(twist_object)
                rospy.sleep(3)
                num = 2
                loop_once = 0


    def clean_up(self):
        cv2.destroyAllWindows()
    


if __name__ == '__main__':
    rospy.init_node('gazebo_scan_sub')
    tobj = LineFollower()
    tobj.wall_avoid()

    rospy.spin()
