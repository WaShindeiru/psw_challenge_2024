import rospy
from geometry_msgs.msg import Twist, Pose
import numpy as np
import time



def quaternion2euler(qx, qy, qz, qw):
    rx = np.arctan2(qw*qx + qy*qz, 1-2*(qx**2+qy**2))
    ry = -(np.pi/2) + 2*np.arctan2(np.sqrt(1+2*(qw*qy-qx*qz)), np.sqrt(1-2*(qw*qy-qx*qz)))
    rz = np.arctan2(2*(qw*qz + qx*qy), 1-2*(qy**2+qz**2))
    return (rx, ry, rz)


def euler2quaternion(rx, ry, rz):
    cr = np.cos(rx * 0.5)
    sr = np.sin(rx * 0.5)
    cp = np.cos(ry * 0.5)
    sp = np.sin(ry * 0.5)
    cy = np.cos(rz * 0.5)
    sy = np.sin(rz * 0.5)

    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    
    return (qx, qy, qz, qw)


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

        self.pub.publish(val_to_send)

    def stop(self):
        self.update_speed(0, 0, 0, 0, 0, 0)

class PositionController:
    def __init__(self):
        self.pos_sub = PositionSubscriber()
        self.speed_pub = SpeedPublisher()
        self.in_destination = False
        self.finished_rotating = False

    def print_rot(self):
        current_pos = self.pos_sub.get_ext_pos_dict()
        current_euler = quaternion2euler(current_pos['rx'], current_pos['ry'], current_pos['rz'], current_pos['w'])
        current_euler = [round(euler, 2) for euler in current_euler]
        print(f"current euler rot: {current_euler}")

    def update_speed(self, x_speed=None, y_speed=None, z_speed=None):
        current_pos = self.pos_sub.get_ext_pos_dict()

        current_rx = current_pos['rx']
        current_ry = current_pos['ry']
        current_rz = current_pos['rz']

        # x_speed_new = x_speed * np.cos(current_rz) - y_speed * np.sin(current_rz)
        # y_speed_new = x_speed * np.sin(current_rz) + y_speed * np.cos(current_rz)
        # ???
        x_speed_new = x_speed
        y_speed_new = x_speed
        z_speed_new = z_speed

        self.speed_pub.update_speed(x=x_speed_new, y=y_speed_new , z=z_speed_new)


    def fly_to_vec(self, vec):
        self.fly_to(vec[0], vec[1], vec[2])

    def rotate(self, rx, ry, rz):
        pos_dict = self.pos_sub.get_ext_pos_dict()
        current_qx = pos_dict['rx']
        current_qy = pos_dict['ry']
        current_qz = pos_dict['rz']
        current_qw = pos_dict['w']

        current_euler = quaternion2euler(current_qx, current_qy, current_qz, current_qw)
        current_euler = [round(euler, 2) for euler in current_euler]
        print(f"current euler rot: {current_euler}")

        desired_quaternion = euler2quaternion(rx, ry, rz)

        x_diff = desired_quaternion[0] - current_qx 
        y_diff = desired_quaternion[1] - current_qy 
        z_diff = desired_quaternion[2] - current_qz 
        w_diff = desired_quaternion[3] - current_qw 

        euler_diffs = quaternion2euler(x_diff, y_diff, z_diff, w_diff)
        euler_diffs = [euler if abs(euler) < 3.12 else 0 for euler in euler_diffs ]
        print(f"euler diffs: {euler_diffs}")

        tolerance = 0.2
        if abs(euler_diffs[0]) < tolerance and abs(euler_diffs[1]) < tolerance and abs(euler_diffs[2]) < tolerance:
            self.finished_rotating = True
        else:
            self.finished_rotating = False

        k = 0.2
        x_speed = euler_diffs[0] * k
        y_speed = euler_diffs[1] * k
        z_speed = euler_diffs[2] * k

        self.speed_pub.update_speed(rx=x_speed, ry=y_speed, rz=z_speed)

    # fly me to the moon
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

        time.sleep(1)

        while not rospy.is_shutdown():

            if not controller.finished_rotating:
                controller.rotate(rx=0, ry=0, rz=0)
            
            if controller.finished_rotating:
                print("finished")
                controller.print_rot()
                controller.update_speed(0, 0, 0)
                # if controller.in_destination:
                #     current_point = next(points_iter)
                # controller.fly_to_vec(current_point)
            rate.sleep()
    except rospy.ROSInterruptException:
        pass
    except StopIteration:
        controller.stop()
