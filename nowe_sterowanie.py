import rospy
from geometry_msgs.msg import Twist, Pose

class PositionSubscriber:
    def __init__(self):
        self.sub = rospy.Subscriber("/iris_control/pose", Pose, self._update_pos)
        self.pos = {'x': 0.0,
                    'y': 0.0,
                    'z': 0.0,
                    'rx': 0.0,
                    'ry': 0.0,
                    'rz': 0.0}
        self.ext_pos = {**self.pos,
                        'w': 0.0}

    def _update_pos(self, data):
        position = data.position
        rotation = data.orientation

        print(position)

        self.pos = {'x': position.x,
                    'y': position.y,
                    'z': position.z,
                    'rx': rotation.x,
                    'ry': rotation.y,
                    'rz': rotation.z}
        
        self.ext_pos = {**self.pos,
                        'w': rotation.w}
        
    def get_pos_dict(self):
        return self.pos
    
    def get_ext_pos_dict(self):
        return self.ext_pos  
    
    def get_position_vec(self):
        return [self.pos['x'], self.pos['y'], self.pos['z']]
    
    def get_rotation_vec(self):
        return [self.ext_pos['rx'], self.ext_pos['ry'], self.ext_pos['rz'], self.ext_pos['w']]
    

class SpeedPublisher:
    def __init__(self):
        self.pub = rospy.Publisher('/iris_control/cmd_vel', Twist, queue_size=10)

    def update_speed(self, x=None, y=None, z=None, rx=None, ry=None, rz=None):
        val_to_send = Twist()

        val_to_send.linear.x = x if x else 0.0
        val_to_send.linear.y = y if y else 0.0
        val_to_send.linear.z = z if z else 0.0

        val_to_send.angular.x = rx if rx else 0.0
        val_to_send.angular.y = ry if ry else 0.0
        val_to_send.angular.z = rz if rz else 0.0

        # rospy.loginfo(val_to_send)
        self.pub.publish(val_to_send)

    def stop(self):
        self.update_speed(0, 0, 0, 0, 0, 0)

class PositionController:
    def __init__(self):
        self.pos_sub = PositionSubscriber()
        self.speed_pub = SpeedPublisher()
        self.in_destination = False

    def fly_to_vec(self, vec):
        self.fly_to(vec[0], vec[1], vec[2])

    def fly_to(self, x=None, y=None, z=None):
        pos_dict = self.pos_sub.get_pos_dict()
        current_x = pos_dict['x']
        current_y = pos_dict['y']
        current_z = pos_dict['z']

        x_diff = x - current_x
        y_diff = y - current_y
        z_diff = z - current_z

        if abs(x_diff) < 0.3 and abs(y_diff) < 0.3 and abs(z_diff) < 0.3:
            self.in_destination = True
        else:
            self.in_destination = False

        k = 0.2
        x_speed = x_diff * k
        y_speed = y_diff * k
        z_speed = z_diff * k

        self.speed_pub.update_speed(x_speed, y_speed, z_speed, 0, 0, 0)

    def stop(self):
        self.speed_pub.stop()


if __name__ == '__main__':
    try:
        rospy.init_node('iris_node', anonymous=True)
        # speed_pub = SpeedPublisher()
        # pos_sub = PositionSubscriber()
        controller = PositionController()

        rate = rospy.Rate(10) # 10hz

        points = [(0, 0, 5),
                  (10, 0, 5),
                  (10, 10, 5),
                  (0, 10, 5),
                  (0, 0, 5)]
        
        points_iter = iter(points)

        current_point = points[0]
        while not rospy.is_shutdown():
            if controller.in_destination:
                current_point = next(points_iter)

            controller.fly_to_vec(current_point)
            
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
