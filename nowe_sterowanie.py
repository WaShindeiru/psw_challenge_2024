import rospy
from geometry_msgs.msg import Twist, Pose
import numpy as np

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

        # print(position)

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

        rospy.loginfo(val_to_send)
        self.pub.publish(val_to_send)

    def stop(self):
        self.update_speed(0, 0, 0, 0, 0, 0)

class PositionController:
    def __init__(self):
        self.pos_sub = PositionSubscriber()
        self.speed_pub = SpeedPublisher()
        self.in_destination = False
        self.finished_rotating = False

    def update_speed(self, x_speed=None, y_speed=None, z_speed=None):
        current_pos = self.pos_sub.get_ext_pos_dict()

        current_rx = current_pos['rx']
        current_ry = current_pos['ry']
        current_rz = current_pos['rz']

        x_speed_new = x_speed * np.cos(current_rz) - y_speed * np.sin(current_rz)
        y_speed_new = x_speed * np.sin(current_rz) + y_speed * np.cos(current_rz)
        z_speed_new = z_speed

        self.speed_pub.update_speed(x=x_speed_new, y=y_speed_new , z=z_speed_new)


    def fly_to_vec(self, vec):
        self.fly_to(vec[0], vec[1], vec[2])

    def rotate(self, rx=None, ry=None, rz=None):
        pos_dict = self.pos_sub.get_pos_dict()
        current_x = pos_dict['rx']
        current_y = pos_dict['ry']
        current_z = pos_dict['rz']

        x_diff = rx - current_x if rx else 0
        y_diff = ry - current_y if ry else 0
        z_diff = rz - current_z if rz else 0

        tolerance = 0.05
        if abs(x_diff) < tolerance and abs(y_diff) < tolerance and abs(z_diff) < tolerance:
            self.finished_rotating = True
        else:
            self.finished_rotating = False

        k = 0.2
        x_speed = x_diff * k
        y_speed = y_diff * k
        z_speed = z_diff * k

        # print(f"cur_z {current_z}, set_z {rz}, diff {z_diff}, speed {z_speed}")

        self.speed_pub.update_speed(rx=x_speed, ry=y_speed, rz=z_speed)

    def fly_to(self, x=None, y=None, z=None):
        pos_dict = self.pos_sub.get_pos_dict()
        current_x = pos_dict['x']
        current_y = pos_dict['y']
        current_z = pos_dict['z']

        x_diff = x - current_x
        y_diff = y - current_y
        z_diff = z - current_z

        tolerance = 1
        if abs(x_diff) < tolerance and abs(y_diff) < tolerance and abs(z_diff) < tolerance:
            self.in_destination = True
        else:
            self.in_destination = False

        k = 0.2
        x_speed = x_diff * k
        y_speed = y_diff * k
        z_speed = z_diff * k

        self.update_speed(x_speed, y_speed, z_speed)

    def stop(self):
        self.speed_pub.stop()


if __name__ == '__main__':
    try:
        rospy.init_node('iris_node', anonymous=True)

        controller = PositionController()
        rate = rospy.Rate(10) # 10hz

        # points = [(0, 0, 5),
        #           (10, 0, 5),
        #           (10, 10, 5),
        #           (0, 10, 5),
        #           (0, 0, 5)]
        
        # points_iter = iter(points)

        # current_point = points[0]
        while not rospy.is_shutdown():

            if not controller.finished_rotating:
                print(controller.finished_rotating)
                controller.rotate(rz=1)
            
            if controller.finished_rotating:
                controller.update_speed(0.8, 0, 0)


                # if controller.in_destination:
                #     current_point = next(points_iter)
                # controller.fly_to_vec(current_point)


            
            
            rate.sleep()
    except rospy.ROSInterruptException:
        pass
    except StopIteration:
        controller.stop()
