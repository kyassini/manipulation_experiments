###################################################################
# Camera Class - Subscribes to real arm depth and color camera    #
# Publishes depth as an array, segmented image, and a color image #                                      # 
###################################################################

import rospy
import cv2
import numpy as np

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class Camera:
    def __init__(self):
        """ Camera topics, note: real arm has a different depth topic """
        rospy.Subscriber("/camera/depth/image", Image, self.depth_img, queue_size=1)
        rospy.Subscriber("/camera/color/image_raw", Image, self.binary_img, queue_size=1)

        self.depth_pub = rospy.Publisher("image_depth", Image, queue_size=1)
        self.seg_pub = rospy.Publisher("image_seg", Image, queue_size=1)

        self.bridge = CvBridge()
        self.depth = None
        self.bin_img = None
        self.color_img = None
 
    def depth_img(self, msg):
        # convert to a ROS image
        cv_img = self.bridge.imgmsg_to_cv2(msg, "32FC1")
        height, width = cv_img.shape
        #self.depth_img = cv_img
        #scale_per = 60
        #width = int(cv_img.shape[1] * scale_per/100)
        #height = int(cv_img.shape[0] * scale_per/100)
        #dim = (width, height)
        #self.depth_img = cv2.resize(cv_img, dim, interpolation=cv2.INTER_AREA)

        self.depth_img = cv_img

        #self.depth_img = cv_img[55:height, 45:width]
        #self.depth_img = self.img_cropper(self.depth_img)

        #self.depth_img = cv2.resize(self.depth_img, ((1280,720)))

        np_img = np.asarray(self.depth_img)
        self.depth_arr = np_img.astype(float)

    def binary_img(self, msg):
        cv_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        height, width,_ = cv_img.shape

        self.color_img = cv2.resize(cv_img, ((480,270)))
        gray = cv2.cvtColor(self.color_img,cv2.COLOR_BGR2GRAY)
        ret, thresh = cv2.threshold(gray,0,255,cv2.THRESH_BINARY_INV+cv2.THRESH_OTSU)

        self.bin_img = self.img_cropper(thresh)

    def img_cropper(self, img):
        try:
            height, width = img.shape
        except:
            height, width,_= img.shape
        img[height-100:height, :] = 0 # Real crop
        #img[:, width-400:width] = 0    # sim crop
        return img

    def get_imgs(self):   
        np.save('/home/kyassini/Desktop/depth_imgs/depth.npy', self.depth)
        cv2.imwrite('/home/kyassini/Desktop/depth_imgs/seg.png', self.bin_img)

        #self.color_img= cv2.resize(self.color_img, ((480,270)))
        #self.bin_img= cv2.resize(self.bin_img, ((480,270)))

        #rospy.logerr(self.color_img.shape)


        rospy.logwarn("Getting imgs...")
        ros_depth = self.bridge.cv2_to_imgmsg(self.depth_arr, "64FC1")
        ros_depth_img = self.bridge.cv2_to_imgmsg(self.depth_img, "32FC1")
        ros_bin_img = self.bridge.cv2_to_imgmsg(self.bin_img, "8UC1")
        ros_color_img = self.bridge.cv2_to_imgmsg(self.color_img, "bgr8")

        self.seg_pub.publish(ros_color_img)
        self.depth_pub.publish(ros_depth_img)

        return ros_depth, ros_bin_img, ros_color_img

        #ros_bin_img = self.bridge.cv2_to_imgmsg(self.bin_img, "mono8")
        #self.depth_pub.publish(self.depth)
        #self.seg_pub.publish(ros_bin_img)