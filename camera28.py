import numpy as np
import matplotlib.pyplot as plt
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class CameraSubscriber:
    def __init__(self):
        self.sub = rospy.Subscriber("/iris/usb_cam/image_raw", Image, self.new_image_callback)
        self.bridge = CvBridge()
        self.img = None

    def new_image_callback(self, img_data):
        self.img = cv2.flip(np.asarray(cv2.cvtColor(self.bridge.imgmsg_to_cv2(img_data), cv2.COLOR_BGR2GRAY)),-1)

class ArucoDetection:
    def __init__(self):
        self.this_aruco_dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
        self.this_aruco_parameters = cv2.aruco.DetectorParameters_create()

    def detect(self, frame):
        (corners, ids, rejected) = cv2.aruco.detectMarkers(
        frame, self.this_aruco_dictionary, parameters=self.this_aruco_parameters)
        print(ids)

        frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2RGB)

        x,y,z = frame.shape
        cv2.circle(frame, (y//2, x//2), 3, (255,0,0), 2)

        if len(corners) > 0:
            # Flatten the ArUco IDs list
            ids = ids.flatten()
            
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
                print(f"aruco size : {aruco_size_px}")
                default_aruco_size_px = 50

                calculated_distance = aruco_size_px/default_aruco_size_px

                center_gate_default_px = 120

                # Our desired destination
                cv2.circle(frame, (int(top_left[0]-calculated_distance*center_gate_default_px), int(top_left[1]-calculated_distance*center_gate_default_px)), 3, (0, 255, 0), 2)
                
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

        cv2.imshow("y",frame)
        cv2.imwrite("image_camera.jpg", frame)
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
