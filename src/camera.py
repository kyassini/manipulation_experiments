#!/usr/bin/env python

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class Camera:
    def __init__(self):
        rospy.Subscriber("/camera/depth/image_raw", Image, self.depth_img)
        rospy.Subscriber("/camera/color/image_raw", Image, self.binary_img)

        self.depth_pub = rospy.Publisher("image_depth", Image, queue_size=10)
        self.seg_pub = rospy.Publisher("image_seg", Image, queue_size=10)

        self.bridge = CvBridge()
        self.depth = None
        self.bin_img = None
        self.color_img = None
 
    def depth_img(self, msg):
        # convert to a ROS image
        cv_img = self.bridge.imgmsg_to_cv2(msg, "32FC1")
        np_img = np.asarray(cv_img)
        self.depth = np_img.astype(float)

    def binary_img(self, msg):
        cv_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        self.color_img = cv_img

        gray = cv2.cvtColor(cv_img,cv2.COLOR_BGR2GRAY)
        #gray = self.img_cropper(gray)
        ret, thresh = cv2.threshold(gray,0,255,cv2.THRESH_BINARY_INV+cv2.THRESH_OTSU)

        self.bin_img = self.img_cropper(thresh)

    def img_cropper(self, img):
        height, width = img.shape
        img[:, width-400:width] = 0
        return img

    def get_imgs(self):   
        #np.save('/home/kyassini/Desktop/depth_imgs/depth.npy', self.depth)
        #cv2.imwrite('/home/kyassini/Desktop/depth_imgs/seg.png', self.bin_img)


        ros_depth = self.bridge.cv2_to_imgmsg(self.depth, "64FC1")
        ros_bin_img = self.bridge.cv2_to_imgmsg(self.bin_img, "8UC1")
        ros_color_img = self.bridge.cv2_to_imgmsg(self.color_img, "bgr8")

        self.seg_pub.publish(ros_bin_img)

        return ros_depth, ros_bin_img, ros_color_img

        #ros_bin_img = self.bridge.cv2_to_imgmsg(self.bin_img, "mono8")
        #self.depth_pub.publish(self.depth)
        #self.seg_pub.publish(ros_bin_img)