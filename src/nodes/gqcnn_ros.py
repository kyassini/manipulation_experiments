#!/usr/bin/env python

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class GQCNN_imgs:
    def __init__(self):
        rospy.Subscriber("/camera/depth/image_raw", Image, self.depth_img)
        rospy.Subscriber("/camera/color/image_raw", Image, self.binary_img)

        self.pub = rospy.Publisher("image_depth", Image, queue_size=10)
        self.bridge = CvBridge()

    def depth_img(self, msg):
        # convert to a ROS image
        cv_img = self.bridge.imgmsg_to_cv2(msg, "32FC1")
        np_img = np.asarray(cv_img)
        img = np_img.astype(float)

        np.save('/home/kyassini/Desktop/depth_imgs/depth.npy', img)

        # convert new image to ROS
        #ros_flipped = self.bridge.cv2_to_imgmsg(cv_img, "32FC1")

        # publish cropped image
        #self.pub.publish(msg)

    def binary_img(self, msg):
        cv_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        
        gray = cv2.cvtColor(cv_img,cv2.COLOR_BGR2GRAY)
        #gray = self.img_cropper(gray)
        ret, thresh = cv2.threshold(gray,0,255,cv2.THRESH_BINARY_INV+cv2.THRESH_OTSU)

        img = self.img_cropper(thresh)
        cv2.imwrite('/home/kyassini/Desktop/depth_imgs/seg.png', thresh)
        
        cv2.imshow('y1o', gray)
        cv2.imshow('yo', thresh)
        cv2.waitKey(0)

    def img_cropper(self, img):
        height, width = img.shape
        img[:, width-400:width] = 0
        return img



if __name__ == "__main__":
   rospy.init_node("gqcnn_ros", anonymous=True)

   imgs = GQCNN_imgs()

   rospy.spin()