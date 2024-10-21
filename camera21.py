import numpy as np
import matplotlib.pyplot as plt
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class CameraSubscriber:
    def __init__(self):
        self.sub = rospy.Subscriber("/iris/usb_cam/image_raw", Image, self._update_image)
        self.bridge = CvBridge()
        self.img = None

    def _update_image(self, img_data):
        img_msg = self.bridge.imgmsg_to_cv2(img_data)
        img = np.asarray(img_msg)
        self.img = img




if __name__ == '__main__':
    rospy.init_node('iris_node', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    cam_sub = CameraSubscriber()

    while not rospy.is_shutdown():
        if cam_sub.img is not None:
            cv2.imshow("y",cam_sub.img)
            cv2.waitKey(1)

        rate.sleep()
