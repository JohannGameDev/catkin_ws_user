#!/usr/bin/env python
# -*- coding: utf-8 -*-
import roslib
# roslib.load_manifest('my_package')
import sys
import rospy
import cv2
from std_msgs.msg import String
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from math import sqrt
from std_msgs.msg import UInt8
from std_msgs.msg import Int16
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from line_path import LinePath
from datetime import datetime


class image_converter:
    def __init__(self):

        # --- definitions ---
        self.epsilon = 0.05  # allowed inaccuracy for distance calculation
        self.speed_rpm = 150
        # angles are 180 degress from 0 (strong left) to 90 (straight) to 180 (strong right)
        self.angle_left = 75 #
        self.angle_straight = 90
        self.angle_right = 105
        self.last_odom = None
        self.is_active = False

        self.last_time = datetime.now()

        # Keine Ahnung
        # create subscribers and publishers
        #self.sub_odom = rospy.Subscriber("odom", Odometry, self.callbackOdom, queue_size=100)
        # wait for first odometry message, till adverting subscription of commands
        #self.waitForFirstOdom()
        self.pub_stop_start = rospy.Publisher(
            "manual_control/stop_start",
            Int16,
            queue_size=100)
        self.pub_speed = rospy.Publisher("manual_control/speed", Int16, queue_size=100)
        self.pub_steering = rospy.Publisher(
            "steering",
            UInt8,
            queue_size=100)# steering oder steering_angle???
        self.pub_speed.publish(0)  # speed zero
        self.pub_stop_start.publish(1)
        self.pub_info = rospy.Publisher("simple_drive_control/info", String, queue_size=100)

        rospy.loginfo(rospy.get_caller_id() + ": started!")



        self.image_pub = rospy.Publisher("/image_processing/bin_line", Image, queue_size=1)

        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.callback, queue_size=1)
        self.line_path = LinePath()

        self.sub_forward = rospy.Subscriber("simple_drive_control/forward", Float32, self.callbackForward, queue_size=10)

        self.sub_forward_left = rospy.Subscriber("simple_drive_control/forward_left", Float32, self.callbackForwardLeft, queue_size=10)
        self.sub_forward_right = rospy.Subscriber("simple_drive_control/forward_right", Float32, self.callbackForwardRight, queue_size=10)

        self.pub_back_left = rospy.Publisher("simple_drive_control/backward_left",Float32,queue_size=10)
        self.pub_back_right = rospy.Publisher("simple_drive_control/backward_right",Float32,queue_size=10)

        self.pub_forward = rospy.Publisher("simple_drive_control/forward",Float32,queue_size=10)
        rospy.loginfo(rospy.get_caller_id() + ": about to publsih")

        self.drive(0.5,"somecommand",self.speed_rpm,self.angle_straight)
        #rospy.loginfo(rospy.get_caller_id() + ": published")


    def callback(self, data):
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

            sensitivity = 80
            lower_white = np.array([0, 0, 255 - sensitivity])
            upper_white = np.array([255, sensitivity, 255])
            # Threshold the HSV image to get only blue colors
            mask = cv2.inRange(hsv, lower_white, upper_white)
            cv2.imwrite('src/assignment6/data/og_pic.png', cv_image)

            # Bitwise-AND mask and original image
            image = cv2.bitwise_and(cv_image, cv_image, mask=mask)
            # cv2.imwrite('src/assignment5/data/test_mask.png', res)
            # An dem Punkt haben wir ein Schwarz weiß bild mit der weißen linie, aber es ist in wirklichkeit noch bgr2
            line_pic,middle_point = self.line_path.get_image_line(image)
            cv2.imwrite('src/assignment6/data/pic_with_line_eq.png', line_pic)

            self.image_pub.publish(self.bridge.cv2_to_imgmsg(line_pic, "bgr8"))

            current_time = datetime.now()
            if (current_time - self.last_time).seconds > 1:
                rospy.loginfo(
                    "%s: update",
                    rospy.get_caller_id())
                self.last_time = datetime.now()
                if (middle_point[1] > self.line_path.xLen/2):
                    self.pub_steering.publish(self.angle_left)
                else:
                    self.pub_steering.publish(self.angle_right)


        except CvBridgeError as e:
            print(e)

    def callbackOdom(self,msg):
        self.last_odom = msg

    def waitForFirstOdom(self):
        while not rospy.is_shutdown() and self.last_odom is None:
            rospy.loginfo(
                "%s: No initial odometry message received. Waiting for message...",
                rospy.get_caller_id())
            rospy.sleep(1.0)

    def callbackForward(self,msg):
        rospy.loginfo(rospy.get_caller_id() + ":Drive forward ")
        self.drive(msg.data, "callbackForward", self.speed_rpm, self.angle_straight)

    def callbackForwardLeft(self,msg):
        self.drive(msg.data, "callbackForwardLeft", self.speed_rpm, self.angle_left)

    def callbackForwardRight(self,msg):
        self.drive(msg.data, "callbackForwardRight", self.speed_rpm, self.angle_right)

    def drive(self,distance, command, speed, angle):

        rospy.loginfo("%s: Running %s(%f),speed:(%f),angle:(%f)", rospy.get_caller_id(), command, distance,speed,angle)
        if distance <= 0:
            rospy.logerr(
                "%s: Error, distance argument has to be > 0! %f given",
                rospy.get_caller_id(),
                distance)
            return

        self.pub_info.publish("BUSY")
        if self.is_active:
            rospy.logwarn(
                "%s: Warning, another command is still active! Please wait and try again.",
                rospy.get_caller_id())
            return

        self.is_active = True
        rospy.loginfo("START DRIVING!!! ")

        # stop the car and set desired steering angle + speed
        self.pub_speed.publish(0) #  speed zero
        self.pub_stop_start.publish(1)
        rospy.sleep(1)
        self.pub_steering.publish(angle)
        self.pub_stop_start.publish(0)
        rospy.sleep(1)
        self.pub_speed.publish(speed)
        rospy.loginfo("IAM DRIVING ")

        # start_pos = self.last_odom.pose.pose.position
        # current_distance = 0
        # # check wheather distance is drived
        # while not rospy.is_shutdown() and current_distance < (distance - self.epsilon):
        #     current_pos = self.last_odom.pose.pose.position
        #     current_distance = sqrt(
        #         (current_pos.x - start_pos.x) ** 2 + (current_pos.y - start_pos.y) ** 2)
        #     # rospy.loginfo("current distance = %f", current_distance)
        #     rospy.sleep(0.1)
        #
        # self.pub_speed.publish(0)
        # self.is_active = False
        # current_pos = self.last_odom.pose.pose.position
        # current_distance = sqrt((current_pos.x - start_pos.x)
        #                         ** 2 + (current_pos.y - start_pos.y) ** 2)
        # self.pub_info.publish("FINISHED")
        #
        # rospy.loginfo(
        #     "%s: Finished %s(%f)\nActual travelled distance = %f",
        #     rospy.get_caller_id(),
        #     command,
        #     distance,
        #     current_distance)

def main(args):
    rospy.init_node('image_converter', anonymous=True)
    ic = image_converter()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
        ic.pub_speed.publish(0)  # speed zero
        ic.pub_stop_start.publish(1)
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)
