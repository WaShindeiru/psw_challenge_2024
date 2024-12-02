import rospy
import time
import numpy as np

from aruco_detection import ArucoDetection, CameraSubscriber
from uwu_fligt import PositionController

from enum import Enum

class State(Enum):
    FlyZero = 0
    FlyAruco = 1
    FlyThrough = 2
    Wait = 3

def quaternion2euler(quaternion_rot_vector):
    qx, qy, qz, qw = quaternion_rot_vector
    rx = np.arctan2(2*(qw*qx + qy*qz), 1-2*(qx**2+qy**2))
    ry = -(np.pi/2) + 2*np.arctan2(np.sqrt(1+2*(qw*qy-qx*qz)), np.sqrt(1-2*(qw*qy-qx*qz)))
    rz = np.arctan2(2*(qw*qz + qx*qy), 1-2*(qy**2+qz**2))
    return (rx, ry, rz)






if __name__ == '__main__':

    FRAME_WIDTH = 1
    try:
        rospy.init_node('iris_node', anonymous=True)
        controller = PositionController()
        cam_sub = CameraSubscriber()
        arucoDetection = ArucoDetection()
        rate = rospy.Rate(10) # 10hz
        time.sleep(1)
        

        current_state = State.FlyZero
        prev_state = current_state
        current_marker = 1

        print(f"Started with state: {current_state}")
        while not rospy.is_shutdown():
            if current_state == State.FlyZero:
                controller.fly_to([-2, -2, 2.5], [0, 0, 0])

                if controller.in_destination and controller.finished_rotating:
                    current_state = State(current_state.value + 1)

            if current_state == State.FlyAruco:
                x_diff = -0.4
                if cam_sub.img is not None:
                    arucoDetection.detect(cam_sub.img)
                    result = arucoDetection.markers.get(current_marker, None)
                    next_aruco = arucoDetection.markers.get(current_marker+1, None)
                    print(f"markers: {arucoDetection.markers.keys()} current_marker: {current_marker}, state: {current_state}")
                    next_aruco_direction = None
                    if result is not None:
                        x, y = result
                        x_diff = ((640 - x) * 0.05 ) * (np.pi/180)
                        #print(f"x_diff: {x_diff}")
                        if next_aruco is not None:
                            x_next, y_next = next_aruco
                            x_diff_next = 640 - x_next
                            next_aruco_direction = -1 if x_diff_next > 0 else 1   # -1 is left, 1 is right
                    #else:
                        
                        # if next_aruco_direction is not None:
                        #     print('test')
                        #     x_diff = 0.2 * next_aruco_direction
                        #else:
                        #    x_diff = 0

                direction = [0, 0, 2.5]
                print(f'Rotation: {x_diff}')
                controller.fly_local(direction, x_diff)

                if np.abs(x_diff) < 0.05:
                    #print("Finished calibration")
                    current_quaternion = controller.pos_sub.get_rot_quaternion()
                    current_euler_z = quaternion2euler(current_quaternion)[2]
                    #print(f'Yaw: {current_euler_z / np.pi * 180.0}')
                    x_new ,y_new, _ = controller.pos_sub.get_pos()
                    # a = np.tan(current_euler_z)
                    # b = current_point[1] - a * current_point[0]
                    length = 5
                    y_new += np.sin(current_euler_z) * length   #current_point[0] + 5
                    x_new += np.cos(current_euler_z) * length   #y_new = a * x_new + b

                    new_point = [x_new, y_new, 2.5]
                    #print(new_point)
                    cnt = 0
                    current_state = State(current_state.value + 1)

            if current_state == State.FlyThrough:
                if cam_sub.img is not None:
                    arucoDetection.detect(cam_sub.img)
                    result = arucoDetection.markers.get(current_marker, None)
                    print(f"markers: {arucoDetection.markers.keys()} current_marker: {current_marker}, state: {current_state}")


                    if result is not None:
                        controller.fly_to(new_point, current_euler_z, k=0.7) #fly_local(direction, 0)
                    else:
                        cnt += 1
                        
                        if cnt > 2:
                            current_marker += 1
                            if current_marker > 5:
                                current_state = State.Wait
                                print("Finished flying through")
                            else:
                                current_state = State.FlyAruco
                

            if current_state == State.Wait:
                arucoDetection.detect(cam_sub.img)
                direction = [-5 , 0, 2.8]
                controller.fly_local(direction, 0)
                # time.sleep(2)
                # current_state = State.FlyAruco

            #if prev_state != current_state:
            #print(f"current state {current_state}")

            prev_state = current_state
            rate.sleep()

    except KeyboardInterrupt:
        controller.stop()
    except rospy.ROSInterruptException:
        controller.stop()
