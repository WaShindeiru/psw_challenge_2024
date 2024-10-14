import rospy
from geometry_msgs.msg import Twist


class SpeedPublisher:
    def __init__(self):
        rospy.init_node('iris_node', anonymous=True)
        self.pub = rospy.Publisher('/iris_control/cmd_vel', Twist, queue_size=10)

    def update_speed(self, x=None, y=None, z=None, rx=None, ry=None, rz=None):
        val_to_send = Twist()

        val_to_send.linear.x = x if x else 0.0
        val_to_send.linear.y = y if y else 0.0
        val_to_send.linear.z = z if z else 0.0

        val_to_send.angular.x = rx if rx else 0.0
        val_to_send.angular.y = ry if ry else 0.0
        val_to_send.angular.z = rz if rz else 0.0

        rospy.loginfo(val_to_send)
        self.pub.publish(val_to_send)


if __name__ == '__main__':
    try:
        speed_pub = SpeedPublisher()
        rate = rospy.Rate(10) # 10hz

        while not rospy.is_shutdown():
            speed_pub.update_speed(rz=0)
            
            rate.sleep()
    except rospy.ROSInterruptException:
        pass


# if __name__ == '__main__':
#     try:
#         speed_pub = SpeedPublisher()
#         rate = rospy.Rate(10) # 10hz
#         i = 0
#         while not rospy.is_shutdown():
            
#             if i < 50:
#                 speed_pub.update_speed(x=1)
#             elif i < 100:
#                 speed_pub.update_speed(y=1)
#             elif i < 150:
#                 speed_pub.update_speed(x=-1)
#             elif i < 200:
#                 speed_pub.update_speed(y=-1) 
            
#             if i > 200:
#                 i = 0

#             i = i + 1        
#             rate.sleep()
#     except rospy.ROSInterruptException:
#         pass
