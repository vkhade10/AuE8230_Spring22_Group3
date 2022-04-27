#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
#from move_robot import MoveTurtlebot3
from darknet_ros_msgs.msg import BoundingBoxes
traffic_sign_c = 0

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
loop_once = 1
flag =0


class LineFollower(object):

    def __init__(self,pub):
        self.bridge_object = CvBridge()
        self.pub = pub
        self.image_sub = rospy.Subscriber("/camera/rgb/image_raw",Image,self.camera_callback)
        self.stop_sign_subscriber = rospy.Subscriber('/darknet_ros/bounding_boxes' , BoundingBoxes, self.stop_sign_callback)
        #self.moveTurtlebot3_object = MoveTurtlebot3()

    def stop_sign_callback(self,msg):
        global num
        num = 0
        if msg.bounding_boxes[len(msg.bounding_boxes)- 1].id == 11:
            prediction = msg.bounding_boxes
            for box in prediction:
                identified_class=box.Class
                probability = float(box.probability)
                area = abs(box.xmax-box.xmin)*abs(box.ymax-box.ymin)
                print(area,'area')
                print('probability',probability)
                if ((probability >= 0.85) and (area >=6000)):
                    print('stop_sign_detected')
                    num = 1



    def traffic_sign_callback(self,data):
         global traffic_sign_c
         for box in data.bounding_boxes:
             if box.id ==11:
                 traffic_sign_c=1
                 rospy.loginfo('Traffic sign detected')
             if box.id !=11:
                 rospy.loginfo('Sign not detected')

         pub = rospy.Publisher("/stop_sign", Int64, queue_size=10)
         msg = Int64()
         msg.data = traffic_sign_c
         pub.publish(msg)


    def clean_up(self):
        # self.moveTurtlebot3_object.clean_class()
        cv2.destroyAllWindows()

def main():
    rospy.init_node('line_following_node', anonymous=True)
    pub = rospy.Publisher('cmd_vel', Twist, queue_size= 10)
    ts_sub = rospy.Subscriber("/darknet_ros/bounding_boxes",BoundingBoxes,traffic_sign_callback)
    line_follower_object = LineFollower(pub)
    rate = rospy.Rate(10)
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
