#!/usr/bin/env python3

#from distutils.log import error
#from re import T
#from tempfile import tempdir
import rospy
import time
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from darknet_ros_msgs.msg import BoundingBoxes
from decimal import *
import numpy as np
from apriltag_ros.msg import AprilTagDetectionArray
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
print("Here once!!")
# for april tag
april_tag_flag = 0
#angle = 0
#linear_dist = 0
temp = ""
# centroid_flag = 0

class LineFollower(object):

    def __init__(self,pub):
        self.bridge_object = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/rgb/image_raw",Image, self.camera_callback)
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.callback_lidar)
        self.stop_sign_subscriber = rospy.Subscriber('/darknet_ros/bounding_boxes' , BoundingBoxes, self.stop_sign_callback)
        self.pub = pub
        #self.april_sub = rospy.Subscriber('/tag_detections_image', Image, self.april_scan)
        self.sub = rospy.Subscriber('/tag_detections', AprilTagDetectionArray,self.april_callback)
        #self.rate = rospy.Rate(200)
        print("Initialized")
        
    
    #def april_scan(self,data):
        #cv_image = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8")
        #print("aprilscan")
        #cv2.imshow("apriltag", cv_image)
        #cv2.waitKey(1)
    
    
      
    def stop_sign_callback(self,msg):
        
        print("Detecting Stop Sign")
        if msg.bounding_boxes[len(msg.bounding_boxes)- 1].id == 11:
            prediction = msg.bounding_boxes
            for box in prediction:
                identified_class=box.Class
                probability = float(box.probability)
                area = abs(box.xmax-box.xmin)*abs(box.ymax-box.ymin)
                print(area,'area')
                print('probability',probability)
                if ((probability >= 0.85) and (area >=6000) and (identified_class == 'stop sign')): 
                    print('stop_sign_detected')
                    # rospy.sleep(1)
                    self.num = 1
    
    def camera_callback(self, data):
        
        print("CAMERA")
        cv_image = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8")
        height, width, channels = cv_image.shape
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
            self.controller = 0

        if cx !=240 and cy !=320:
            self.controller = 1
            print("controller changed")
            e = cx - height/2 - 3
            e31 = e21
            e21 = e11
            e11 = e
            self.u = self.u + k11*e11 + k21*e21 + k31*e31

        # Draw the centroid in the resultut image
        # cv2.circle(img, center, radius, color[, thickness[, lineType[, shift]]]) 
        cv2.circle(mask,(int(cx), int(cy)), 10,(0,0,255),-1)
        cv2.imshow("Original", cv_image)
        cv2.imshow("MASK", mask)
        cv2.waitKey(1)

    def  april_callback(self,data):
        rospy.loginfo("I'm Here !")
        # global angle, linear_dist, april_tag_flag
        if data.detections:
            self.angle = data.detections[0].pose.pose.pose.position.x        #compensate the width to determine the right x difference
            self.linear_dist = data.detections[0].pose.pose.pose.position.z
            if self.linear_dist < 0.08: 
                self.april_tag_flag = 1
        

    def clean_up(self):
        cv2.destroyAllWindows()


    def callback_lidar(self, data):

        #global right_min, left_min, front_min
        self.right_min = 0.3
        self.left_min = 0.3
        self.front_min = 10
        left = data.ranges[90]
        right = data.ranges[270]
        front = data.ranges[0]
        right_cone = []
        left_cone = []
        front_cone = []
        print("lidarsectoring")

        if right == float("inf"): 
            self.right_min = 0.25
        else: 
            for i in range(300,341):
                right_cone.append(data.ranges[i])

        if left == float("inf"): 
            self.left_min = 0.25
        else: 
            for o in range(20,61):
                left_cone.append(data.ranges[o])

        for p in range(len(data.ranges)):
            if p<20 or p>340:
                front_cone.append(data.ranges[p])
            if len(right_cone)!=0:
                self.right_min = min(right_cone)
            if len(left_cone)!= 0: 
                self.left_min = min(left_cone)

            self.front_min = min (front_cone)

def auto_avoid():
    #global right_min, left_min, front_min,d, det_flag, num, controller, april_tag_flag,temp
    
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 10)
    obj = LineFollower(pub)
    e = (obj.left_min - obj.right_min)
    print(e)
    twist_object = Twist()
    print("AutoAvoid")
    if obj.num == 1:
        print("num")
        twist_object.linear.x = 0
        twist_object.angular.z = 0
        # april_tag_flag = 1
        #self.moveTurtlebot3_object.move_robot(twist_object)
        obj.num = 2
        # self.pub.publish(twist_object)
        rospy.sleep(3)
    
    if obj.controller == 1:
        #global u
        print("control", twist_object.linear.x)
        twist_object.linear.x = 0.1
        twist_object.angular.z =  -obj.u/390
    
    elif obj.april_tag_flag == 1 and obj.controller == 0: #make this global inside april callback
        #global angle, linear_dist
        print("AprilTag baby")      
        
        if obj.linear_dist > 0.05:
            twist_object.linear.x = obj.linear_dist
            twist_object.angular.z = -obj.angle * 10 
            # self.pub.publish(twist_object)
        elif obj.linear_dist < 0.035:
            twist_object.linear.x = -((0.05-obj.linear_dist)/0.05) * 0.1
            twist_object.angular.z = obj.angle * 10 
            # self.pub.publish(twist_object)
        else:                
            twist_object.linear.x = 0
            twist_object.angular.z = -0.1
        print("tag", twist_object.linear.x)    
        #self.pub.publish(twist_object)
    
    elif obj.front_min <= 0.25:
        # error = (left_min - right_min)
        u = e11 = e21 = e31 = 0

        kp = 0.4 * 3
        ki = 0.0
        kd = 0.2
        k1 = kp + ki + kd
        k2 = -kp -2*kd
        k3 = kd
        twist_object.linear.x = 0.1
        e31 = e21
        e21 = e11
        e11 = e
        u = u + k1*e11 + k2*e21 + k3*e31
        twist_object.angular.z = u
        print('moving')

    elif front_min > 0.25 and front_min < 1.5:
        # error = (left_min - right_min)
        u = e11 = e21 = e31 = 0

        kp = 0.4
        ki = 0.0
        kd = 0.2
        k1 = kp + ki + kd
        k2 = -kp -2*kd
        k3 = kd
        twist_object.linear.x = 0.1
        e31 = e21
        e21 = e11
        e11 = e
        u = u + k1*e11 + k2*e21 + k3*e31
        twist_object.angular.z = u
        print('moving')

    else:

        twist_object.linear.x = 0.1
        twist_object.angular.z = 1.2 * e
        print('moving')
    pub.publish(twist_object)

def main():
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 10)
    rospy.init_node('gazebo_scan_sub1', anonymous=True)
    rate = rospy.Rate(5)
    #line_follower_object = LineFollower(pub)
    auto_avoid()   
    ctrl_c = False
    def shutdownhook():
        # Works better than rospy.is_shutdown()
        line_follower_object.clean_up()
        rospy.loginfo("Shutdown time!")
        ctrl_c = True
    rospy.on_shutdown(shutdownhook)
    while not ctrl_c:
        rate.sleep()
        
if __name__ == '__main__':
    main()

