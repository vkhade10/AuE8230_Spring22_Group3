#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from move_robot import MoveTurtlebot3

class LineFollower(object):

    def __init__(self):
        self.bridge_object = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/rgb/image_raw",Image,self.camera_callback)
        self.moveTurtlebot3_object = MoveTurtlebot3()

    def camera_callback(self, data):
        # We select bgr8 because its the OpneCV encoding by default
        cv_image = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8")

        # We get image dimensions and crop the parts of the image we dont need
        height, width, channels = cv_image.shape
        # crop_img = cv_image[int((height/2)+100):int((height/2)+120)][1:int(width)] # original
        # crop_img = cv_image[int(height/2):][1:int(width)] #works
        crop_img = cv_image[1:int(height)][160:480] #works
        #crop_img = cv_image[340:360][1:640]

        # Convert from RGB to HSV
        hsv = cv2.cvtColor(crop_img, cv2.COLOR_BGR2HSV)

        # Define the Yellow Colour in HSV

        """
        To know which color to track in HSV use ColorZilla to get the color registered by the camera in BGR and convert to HSV. 
        """

        # Threshold the HSV image to get only yellow colors
        lower_yellow = np.array([20,100,100])
        upper_yellow = np.array([50,255,255])
        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

        # img = cv2.imread('binary-img.png')
        # gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        img = cv2.GaussianBlur(mask, (7, 7), 0)
        edges = cv2.Canny(img, 100, 220)
        # Detect points that form a line
        lines = cv2.HoughLinesP(edges, 1, np.pi/180, 20)
        # theta = np.arange(0,np.pi,np.pi/180)
        #Draw lines 
        for line in lines:
            x1, y1, x2, y2 = line[0]
            cv2.line(img, (x1, y1), (x2, y2), (0, 0, 255), 5)
    
        # Calculate centroid of the blob of binary image using ImageMoments
        # m = cv2.moments(mask, False)

        # try:
        #     cx, cy = m['m10']/m['m00'], m['m01']/m['m00']
        # except ZeroDivisionError:
        #     cx, cy = height/2, width/2
        
        # Draw the centroid in the resultut image
        # cv2.circle(img, center, radius, color[, thickness[, lineType[, shift]]]) 
        # cv2.circle(mask,(int(cx), int(cy)), 10,(0,0,255),-1)
        # cv2.imshow("Original", cv_image)
        
        cv2.imshow("img", img)
        # cv2.imshow("thresh", thresh)
        # cv2.imshow("result", crop_img)
        # cv2.imshow("Edges", edges)
        cv2.waitKey(1)

        #################################
        ###   ENTER CONTROLLER HERE   ###
        #################################

        twist_object = Twist()
        e = x2 - x1
        # rospy.loginfo('Error is:'+ str(e))
        twist_object.linear.x = 0.1
        twist_object.angular.z =  -e / 720 # we use negative sign here, to turn away from the blob.
        #Otherwise turtlebot will go out of the track.

        rospy.loginfo("ANGULAR VALUE SENT===>"+str(twist_object.angular.z))
        # Make it start turning
        self.moveTurtlebot3_object.move_robot(twist_object)

    def clean_up(self):
        self.moveTurtlebot3_object.clean_class()
        cv2.destroyAllWindows()

def main():
    rospy.init_node('line_following_node', anonymous=True)
    line_follower_object = LineFollower()
    rate = rospy.Rate(5)
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









