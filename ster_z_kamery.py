import rospy
import time
import numpy as np

from aruco_detection import ArucoDetection, CameraSubscriber
from position_controller import PositionController

if __name__ == '__main__':

    FRAME_WIDTH = 1
    try:
        rospy.init_node('iris_node', anonymous=True)

        controller = PositionController()
        cam_sub = CameraSubscriber()
        arucoDetection = ArucoDetection()

        rate = rospy.Rate(10) # 10hz

        time.sleep(1)
        
        rotation = 3.1415
        current_point = [0, -2, 2.5]
        current_marker = 1

        while not rospy.is_shutdown():

            if cam_sub.img is not None:
                arucoDetection.detect(cam_sub.img)
                x,y = arucoDetection.markers.get(1, None)

                x_diff = (1280 - y) * 0.1
                
                controller.speed_pub.update_pos_speed([1, 0, 0])
                controller.rotate([0, 0, x_diff])

                '''
                marker_data = arucoDetection.markers.get(current_marker, None)
                if marker_data is not None:
                    rvecs = np.array(marker_data["rvecs"]).flatten()
                    tvecs = np.array(marker_data["tvecs"]).flatten()
                    print(f"rvecs: {rvecs}, tvecs: {tvecs}")

                    x_rot_mat = np.array([
                        [np.cos(rvecs[2]), -np.sin(rvecs[2]), 0],
                        [np.sin(rvecs[2]), np.cos(rvecs[2]), 0],
                        [0, 0, 1]])
                                         
                    y_rot_mat = np.array([
                        [np.cos(rvecs[1]), 0, np.sin(rvecs[1])],
                        [0, 1, 0],
                        [-np.sin(rvecs[1]), 0], np.cos(rvecs[1])])

                    z_rot_mat = np.array([
                        [1, 0, 0], 
                        [0, np.cos(rvecs[0]), -np.sin(rvecs[0])], 
                        [0, np.sin(rvecs[0]), np.cos(rvecs[0])]])

                    moving_vec = np.array([FRAME_WIDTH/2, FRAME_WIDTH/2, 1]) 
                    '''
                    
            if not controller.finished_rotating:
                controller.rotate([0, 0, rotation])
                controller.print_current_rot()
            
            if controller.finished_rotating:
                controller.print_current_pos()
                controller.fly_to(current_point)
                
            rate.sleep()

    except KeyboardInterrupt:
        controller.stop()
    except rospy.ROSInterruptException:
        controller.stop()
