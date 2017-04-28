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
    thr = cv2.threshold(grey, 0, 255, cv2.THRESH_BINARY)[1]
    rest, contours, hierarchy = cv2.findContours(thr, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    ######################## latest addition
    lst = []
    for contour in contours:
        epsilon = 0.05 * cv2.arcLength(contour, True)
        moment = cv2.moments(contour)
        cx = moment["m10"] / moment['m00']
        cy = moment['m01'] / moment['m00']
        area = cv2.contourArea(contour)
        perimeter = cv2.arcLength(contour, True)
        lst.append([contour, np.array([cx, cy])])

    #################################
    # return contours
    return lst

# left to be tested
def find_right_to_left_transformation(left_image, right_image, rgb_range):
    """find corresponding contours between the two images"""
    left_contours = contour_detector(left_image, rgb_range)
    right_contours = contour_detector(right_image, rgb_range)
    # sort by bounded area
    correspondence_lst = []
    left_clst = []
    right_clst = []
    ## find correspondence
    for left_contour in left_contours:
        for right_contour in right_contours:
            if left_contour[1][0] - right_contour[1][0] < 90 and left_contour[1][0] - right_contour[1][0] > 50 and abs(left_contour[1][1] - right_contour[1][1]) < 10:
                correspondence_lst.append([left_contour[0], right_contour[0]])
                left_clst.append(left_contour[1])
                right_clst.append(right_contour[1])
                break
    left_clst = np.array(left_clst)
    right_clst = np.array(right_clst)
    # # find transformation
    # mapping_mat = left_clst.T.dot(right_clst)
    # find translation vector
    return np.mean(left_clst - right_clst), left_clst, right_clst
    # for l in left_contours:
    #     left_clst.append(l[1])
    # for r in right_contours:
    #     right_clst.append(r[1])
    # left_clst = np.asarray(left_clst)
    # right_clst = np.asarray(right_clst)
    # return left_clst, right_clst









if __name__ == "__main__":
    a = ImageSubscriber()
    print "created image subscriber"