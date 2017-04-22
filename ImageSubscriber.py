import numpy as np
import rospy
from geometry_msgs.msg import PointStamped, Point
from visualization_msgs.msg import Marker
import cv2
import cv_bridge
import numpy as np
import rospy
import scipy.misc
import pickle
import matplotlib.pyplot as plt
import time
# import matplotlib.image as mpimg

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped

class ImageSubscriber:
    # Some constants: image size: 1080 * 1920 * 3

    def __init__(self):
        self.right_image = None
        self.left_image = None
        self.info = {'l': None, 'r': None}
        self.bridge = cv_bridge.CvBridge()


        #========SUBSCRIBERS========#
        # image subscribers
        rospy.init_node('image_saver', anonymous=True)
        rospy.Subscriber("/endoscope/left/image_rect_color", Image,
                         self.left_image_callback, queue_size=1)
        rospy.Subscriber("/endoscope/right/image_rect_color", Image,
                         self.right_image_callback, queue_size=1)
        # info subscribers
        rospy.Subscriber("/endoscope/left/camera_info",
                         CameraInfo, self.left_info_callback)
        rospy.Subscriber("/endoscope/right/camera_info",
                         CameraInfo, self.right_info_callback)

    def left_info_callback(self, msg):
        if self.info['l']:
            return
        self.info['l'] = msg

    def right_info_callback(self, msg):
        if self.info['r']:
            return
        self.info['r'] = msg

    def right_image_callback(self, msg):
        if rospy.is_shutdown():
            return
        self.right_image = self.bridge.imgmsg_to_cv2(msg, "rgb8")

    def left_image_callback(self, msg):
        if rospy.is_shutdown():
            return
        self.left_image = self.bridge.imgmsg_to_cv2(msg, "rgb8")


def contour_detector(image, rgb_range):
    """takes in a standard image(1080 * 1920 * 3 numpy array) 
    and range of RGB channel in list/tuple/numpy array(3(RGB) * 2(lower/upper))
    out put a lists of positions of contour in a single N * 1 numpy array.
    """
    r_range, g_range, b_range = rgb_range
    r_lower_bound, r_upper_bound = r_range
    g_lower_bound, g_upper_bound = g_range
    b_lower_bound, b_upper_bound = b_range
    lower_bound = np.array([r_lower_bound, g_lower_bound, b_lower_bound])
    upper_bound = np.array([r_upper_bound, g_upper_bound, b_upper_bound])
    
    mask = cv2.inRange(image, lower_bound, upper_bound)
    rest = cv2.bitwise_and(image, image, mask = mask)
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (9,9))
    # rest = cv2.morphologyEx(rest, cv2.MORPH_GRADIENT, kernel)
    # rest = cv2.morphologyEx(rest, cv2.MORPH_OPEN, kernel)
    # rest = cv2.morphologyEx(rest, cv2.MORPH_CLOSE, kernel)
    opening = cv2.morphologyEx(rest, cv2.MORPH_OPEN, kernel)
    closing = cv2.morphologyEx(opening, cv2.MORPH_CLOSE, kernel)
    grey = cv2.cvtColor(closing, cv2.COLOR_BGR2GRAY)
    rest, contours, hierarchy = cv2.findContours(grey, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    return contours





if __name__ == "__main__":
    a = ImageSubscriber()
    print "created image subscriber"