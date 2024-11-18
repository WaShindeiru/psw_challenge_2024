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
                controller.fly_to([-2, -2, 2.8], [0, 0, 2.8])

                if controller.in_destination and controller.finished_rotating:
                    current_state = State(current_state.value + 1)

            if current_state == State.FlyAruco:
                x_diff = 2.8
                if cam_sub.img is not None:
                    arucoDetection.detect(cam_sub.img)
                    result = arucoDetection.markers.get(current_marker, None)
                    print(f"markers: {arucoDetection.markers.keys()} current_marker: {current_marker}, state: {current_state}")
                    if result is not None:
                        x, y = result
                        x_diff = ((640 - x) * 0.05 ) *(np.pi/180)

                direction = [0 , 0, 2.8]
                controller.fly_local(direction, x_diff)

                if x_diff< 0.005:
                    current_state = State(current_state.value + 1)

            if current_state == State.FlyThrough:
                if cam_sub.img is not None:
                    arucoDetection.detect(cam_sub.img)
                    result = arucoDetection.markers.get(current_marker, None)
                    print(f"markers: {arucoDetection.markers.keys()} current_marker: {current_marker}, state: {current_state}")


                    if result is not None:
                        direction = [0.3 , 0, 2.8]
                        controller.fly_local(direction, 0)
                    else:
                        current_marker += 1
                        current_state = State.FlyAruco
                

            if current_state == State.Wait:
                direction = [0.3 , 0, 2.8]
                controller.fly_local(direction, 0)
                time.sleep(2)
                current_state = State.FlyAruco

            #if prev_state != current_state:
            #print(f"current state {current_state}")

            prev_state = current_state
            rate.sleep()

    except KeyboardInterrupt:
        controller.stop()
    except rospy.ROSInterruptException:
        controller.stop()
