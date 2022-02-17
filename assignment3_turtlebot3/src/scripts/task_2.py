import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class emergencybrake():

    def __init__(self):
        rospy.init_node('emergency_brake', anonymous=True)
        self.rate = rospy.Rate(10)
        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.scan_sub = rospy.Subscriber('/scan',LaserScan,self.callback)
    vel_msg = Twist()
    rate = rospy.Rate(10)
    stop_dist = 1.0

    def 











if __name__ == '__main__':
    try:
        x = EmBrake()
        x.emergencybrake()
    except rospy.ROSInterruptException:
        pass

