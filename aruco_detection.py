import numpy as np
import matplotlib.pyplot as plt
import rospy
import cv2
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge

aruco_marker_side_length = 0.25
FRAME_WIDTH = 1

class CameraSubscriber:
    def __init__(self):
        self.sub = rospy.Subscriber("/iris/usb_cam/image_raw", Image, self.new_image_callback)
        self.bridge = CvBridge()
        self.img = None

    def new_image_callback(self, img_data):
        self.img = cv2.flip(np.asarray(self.bridge.imgmsg_to_cv2(img_data)), -1)
        

class ArucoDetection:
    def __init__(self):
        self.this_aruco_dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
        self.this_aruco_parameters = cv2.aruco.DetectorParameters_create()
        self.markers = {}
        self.subscriber_camera_info = rospy.Subscriber("/iris/usb_cam/camera_info", CameraInfo, self.camera_info_callback)

    def camera_info_callback(self, msg):
        self.camera_matrix = np.array(msg.K).reshape((3,3))
        self.distortion_coefficients = np.array(msg.D)

    def detect(self, frame):
        frame_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        _, frame_bw = cv2.threshold(frame_gray, 100,255, cv2.THRESH_BINARY)
        (corners, ids, rejected) = cv2.aruco.detectMarkers(
        frame_bw, self.this_aruco_dictionary, parameters=self.this_aruco_parameters, cameraMatrix=self.camera_matrix, distCoeff=self.distortion_coefficients)
       
        x,y,z = frame.shape
        cv2.circle(frame, (y//2, x//2), 3, (255,0,0), 2)

        if len(corners) > 0:
            # Flatten the ArUco IDs list
            ids = ids.flatten()
            markers = {}
            
            # Loop over the detected ArUco corners
            for (marker_corner, marker_id) in zip(corners, ids):
        
                # Extract the marker corners
                corners = marker_corner.reshape((4, 2))
                (top_left, top_right, bottom_right, bottom_left) = corners
                
                # Convert the (x,y) coordinate pairs to integers
                top_right = (int(top_right[0]), int(top_right[1]))
                bottom_right = (int(bottom_right[0]), int(bottom_right[1]))
                bottom_left = (int(bottom_left[0]), int(bottom_left[1]))
                top_left = (int(top_left[0]), int(top_left[1]))

                aruco_size_px = np.abs(top_right[0] - top_left[0])
                value = 2.1

                calculated_distance = aruco_size_px*value

                frame_center_point = int(top_left[0]-calculated_distance), int(top_left[1]-calculated_distance)
                self.markers[marker_id] = frame_center_point

                # Our desired destination
                cv2.circle(frame, (int(top_left[0]-calculated_distance), int(top_left[1]-calculated_distance)), 3, (0, 255, 0), 2)
                
                # Draw the bounding box of the ArUco detection
                cv2.line(frame, top_left, top_right, (0, 255, 0), 2)
                cv2.line(frame, top_right, bottom_right, (0, 255, 0), 2)
                cv2.line(frame, bottom_right, bottom_left, (0, 255, 0), 2)
                cv2.line(frame, bottom_left, top_left, (0, 255, 0), 2)
                
                # Calculate and draw the center of the ArUco marker
                center_x = int((top_left[0] + bottom_right[0]) / 2.0)
                center_y = int((top_left[1] + bottom_right[1]) / 2.0)
                cv2.circle(frame, (center_x, center_y), 4, (0, 0, 255), -1)
                
                # Draw the ArUco marker ID on the video frame
                # The ID is always located at the top_left of the ArUco marker
                cv2.putText(frame, str(marker_id), 
                (top_left[0], top_left[1] - 15),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5, (0, 255, 0), 2)


                '''
                rvecs, tvecs, obj_points = cv2.aruco.estimatePoseSingleMarkers(
                    marker_corner,
                    aruco_marker_side_length,
                    self.camera_matrix,
                    self.distortion_coefficients)
                
                #print(f"tvecs: {tvecs}")
                
                markers[marker_id] = {
                    "rvecs": rvecs,
                    "tvecs": tvecs,
                }

                marker_data = self.markers.get(marker_id, None)
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
                        [-np.sin(rvecs[1]), 0, np.cos(rvecs[1])]])

                    z_rot_mat = np.array([
                        [1, 0, 0], 
                        [0, np.cos(rvecs[0]), -np.sin(rvecs[0])], 
                        [0, np.sin(rvecs[0]), np.cos(rvecs[0])]])

                    moving_vec = np.array([[-FRAME_WIDTH/2, -FRAME_WIDTH/2, 1]]) 
                    # print(f"moving_vec shape: {moving_vec.shape}")
                    # print(f"x_rot_mat shape: {x_rot_mat.shape}")
                    # print(f"y_rot_mat shape: {y_rot_mat.shape}")
                    # print(f"z_rot_mat shape: {z_rot_mat.shape}")

                    moved_point = np.matmul(x_rot_mat, np.matmul(y_rot_mat, np.matmul(x_rot_mat, moving_vec.T)))
                    # moved_point = x_rot_mat * y_rot_mat * z_rot_mat * moving_vec

                    tvecs = tvecs.reshape((3,1))

                    print(f"tvecs shape: {tvecs.shape}")
                    print(f"moved_point shape: {moved_point.shape}")

                    moved_point = tvecs + moved_point

                    print(f"moved_point: {moved_point}")



                    cv2.aruco.drawAxis(frame, self.camera_matrix, self.distortion_coefficients, rvecs, moved_point, 0.05)
                    '''


            
        cv2.imshow("y",frame)
        cv2.waitKey(1)
        
if __name__ == '__main__':
    rospy.init_node('iris_node', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    cam_sub = CameraSubscriber()
    arucoDetection = ArucoDetection()

    while not rospy.is_shutdown():
        if cam_sub.img is not None:
            arucoDetection.detect(cam_sub.img)

        rate.sleep()
