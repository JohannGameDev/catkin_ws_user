#!/usr/bin/env python
import roslib
import sys
import rospy
import cv2
import numpy as np

from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image


class CameraCalibration:
    def __init__(self):
        # Image publisher
        self.image_gray_pub = rospy.Publisher("/image_processing/bin_gray_img", Image, queue_size = 1)
        self.image_black_pub = rospy.Publisher("/image_processing/bin_black_img", Image, queue_size = 1)
        # Image source
        self.image_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.callback, queue_size = 1)
        # OpenCV
        self.bridge = CvBridge()

    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        # Convert to grayscale
        gray_img = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        try:
            self.image_gray_pub.publish(self.bridge.cv2_to_imgmsg(gray_img, "mono8"))
        except CvBridgeError as e:
            print(e)

        # Convert to B/W image
        bi_gray_max = 255
        bi_gray_min = 250
        ret, black_img = cv2.threshold(gray_img, bi_gray_min, bi_gray_max, cv2.THRESH_BINARY)
        try:
            self.image_black_pub.publish(self.bridge.cv2_to_imgmsg(black_img, "mono8"))
        except CvBridgeError as e:
            print(e)



def main(args):
    rospy.init_node('camera_calibration', anonymous = True)
    cc = CameraCalibration()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down.")

if __name__ == '__main__':
  main(sys.argv)