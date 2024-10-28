import rospy
from geometry_msgs.msg import Twist, Pose
import numpy as np
import time

def quaternion2euler(quaternion_rot_vector):
    qx, qy, qz, qw = quaternion_rot_vector
    rx = np.arctan2(qw*qx + qy*qz, 1-2*(qx**2+qy**2))
    ry = -(np.pi/2) + 2*np.arctan2(np.sqrt(1+2*(qw*qy-qx*qz)), np.sqrt(1-2*(qw*qy-qx*qz)))
    rz = np.arctan2(2*(qw*qz + qx*qy), 1-2*(qy**2+qz**2))
    return (rx, ry, rz)

def euler2quaternion(euler_rot_vector):
    rx, ry, rz = euler_rot_vector
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
        self.pos_vec = None
        self.quaternion_rot_vec = None

    def get_pos(self):
        return self.pos_vec
    
    def get_rot_quaternion(self):
        return self.quaternion_rot_vec
    
    def get_rot_euler(self):
        return quaternion2euler(self.quaternion_rot_vec)

    def _update_pos(self, data):
        position = data.position
        rotation = data.orientation

        self.pos_vec = [position.x, position.y, position.z]
        self.quaternion_rot_vec = [rotation.x, rotation.y, rotation.z, rotation.w]


class SpeedPublisher:
    def __init__(self):
        self.pub = rospy.Publisher('/iris_control/cmd_vel', Twist, queue_size=10)

    def update_pos_speed(self, position_vector):
        speed = Twist()
        speed.linear.x = position_vector[0]
        speed.linear.y = position_vector[1]
        speed.linear.z = position_vector[2]
        self.pub.publish(speed)

    def update_rot_speed(self, quaternion_rot_vec):
        speed = Twist()
        speed.angular.x = quaternion_rot_vec[0]
        speed.angular.y = quaternion_rot_vec[1]
        speed.angular.z = quaternion_rot_vec[2]
        self.pub.publish(speed)


class PositionController:
    def __init__(self):
        self.pos_sub = PositionSubscriber()
        self.speed_pub = SpeedPublisher()

        self.finished_rotating = False
        self.in_destination = False

    def print_current_pos(self):
        vec = self.pos_sub.get_pos()
        vec = [round(rot, 2) for rot in vec]
        print(f"current_pos: {vec}")        

    def print_current_rot(self):
        vec = self.pos_sub.get_rot_euler()
        vec = [round(rot, 2) for rot in vec]
        print(f"current_euler: {vec}")

    
    def stop(self):
        self.speed_pub.update_pos_speed([0, 0, 0])

    def rotate(self, euler_rot_vector):
        current_quaternion = self.pos_sub.get_rot_quaternion()
        
        current_euler = quaternion2euler(current_quaternion)
        desired_euler = euler_rot_vector

        diff = np.array(desired_euler) - np.array(current_euler)

        tolerance = 0.05
        if np.linalg.norm(diff) < tolerance:
            self.finished_rotating = True
        else:
            self.finished_rotating = False

        k = 0.2
        new_speeds = diff * k
        self.speed_pub.update_rot_speed(new_speeds)

    def fly_to(self, point):
        current_pos = self.pos_sub.get_pos()
        current_euler_rot = self.pos_sub.get_rot_euler()
        current_z_rot = current_euler_rot[2]

        diff_vec = np.array(point) - np.array(current_pos)
        x_diff = diff_vec[0]
        y_diff = diff_vec[1]
        z_diff = diff_vec[2]

        sin_fi = np.sin(current_z_rot)
        cos_fi = np.cos(current_z_rot)

        new_x_diff = x_diff * cos_fi + y_diff * sin_fi
        new_y_diff = -x_diff * sin_fi + y_diff * cos_fi 
        new_z_diff = z_diff

        diff_vec = np.array([new_x_diff, new_y_diff, new_z_diff])
        destination_dist = np.linalg.norm(diff_vec)

        dist_treshold = 1
        if destination_dist < dist_treshold:
            self.in_destination = True
            return
        else:
            self.in_destination = False

        k = 0.2
        speeds = [new_x_diff * k, new_y_diff * k, new_z_diff * k]
        # print(f"new speeds {speeds}")
        self.speed_pub.update_pos_speed(speeds)


if __name__ == '__main__':
    try:
        rospy.init_node('iris_node', anonymous=True)

        controller = PositionController()
        rate = rospy.Rate(10) # 10hz

        time.sleep(1)
        points = [(0, 0, 5),
                (10, 0, 5),
                (10, 10, 5),
                (0, 10, 5),
                (0, 0, 5)]
        
        rotation = 1.57

        points_iter = iter(points)
        current_point = points[0]

        while not rospy.is_shutdown():

            if not controller.finished_rotating:
                controller.rotate([0, 0, rotation])
                controller.print_current_rot()
            
            if controller.finished_rotating:
                controller.print_current_pos()
                if controller.in_destination:
                    current_point = next(points_iter)
                controller.fly_to(current_point)
                
            rate.sleep()

    except KeyboardInterrupt:
        controller.stop()
    except rospy.ROSInterruptException:
        controller.stop()
    except StopIteration:
        pass







