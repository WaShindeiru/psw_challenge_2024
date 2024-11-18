import rospy
from geometry_msgs.msg import Twist

class FlightController(rospy):
    def __init__(self):
        self.command_pub_ = rospy.Publisher('/iris_control/cmd_vel', Twist, queue_size=10)
        rospy.init_node('flight_controller')

    def rotate(self, rotation_value):
        msg = Twist()
        msg.linear.x = 0
        msg.linear.y = 0
        msg.linear.z = 0
        msg.angular.x = 0
        msg.angular.y = 0
        msg.angular.z = rotation_value
        self.pub.publish(msg)
    
    def fly_straight(self, height_correction, straight_speed):
        msg = Twist()
        msg.linear.x = straight_speed
        msg.linear.y = 0
        msg.linear.z = height_correction
        msg.angular.x = 0
        msg.angular.y = 0
        msg.angular.z = 0
        self.pub.publish(msg)
