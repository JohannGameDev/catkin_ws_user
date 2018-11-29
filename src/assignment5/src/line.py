#!/usr/bin/env python
import roslib
# roslib.load_manifest('my_package')
import sys
import rospy
import cv2
from std_msgs.msg import String
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
#import matplotlib
#matplotlib.use('Agg')
from matplotlib import pyplot as plt
import numpy as np

# from __future__ import print_function

class image_converter:

  def __init__(self):
    self.image_pub = rospy.Publisher("/image_processing/bin_line",Image, queue_size=1)

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/camera/color/image_raw",Image,self.callback, queue_size=1)


  def callback(self,data):

    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)



    try:
      # gauss
      MAX_KERNEL_LENGTH = 2;
      i = 5
      dst = cv2.GaussianBlur(cv_image, (5, 5), 0, 0)
      # Convert BGR to HSV
      hsv = cv2.cvtColor(dst, cv2.COLOR_BGR2HSV)

      #cv2.imwrite('src/assignment5/data/test.png', hsv)
      
      sensitivity = 50 # use hsl
      lower_white = np.array([0, 0, 255 - sensitivity])
      upper_white = np.array([255, sensitivity, 255])
      # Threshold the HSV image to get only white colors
      mask = cv2.inRange(hsv, lower_white, upper_white)

      # Bitwise-AND mask and original image
      res = cv2.bitwise_and(cv_image, cv_image, mask=mask)
      
      #cv2.imwrite('src/assignment5/data/test_mask.png', res)

      self.image_pub.publish(self.bridge.cv2_to_imgmsg(res, "bgr8"))

    except CvBridgeError as e:
      print(e)
      

def main(args):
  rospy.init_node('image_converter', anonymous=True)
  ic = image_converter()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()


if __name__ == '__main__':
  main(sys.argv)
