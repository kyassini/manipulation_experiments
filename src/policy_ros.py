# -*- coding: utf-8 -*-

############################################################
# Modified policy_ros for generating grasp from gen3 depth,#
#       segmented, and color images                        #
# Calls grasp_planner_node action from GQCNN/ros_nodes     #
############################################################

""" 
Original: https://github.com/BerkeleyAutomation/gqcnn/blob/master/examples/policy_ros.py

Copyright ©2017. The Regents of the University of California (Regents).
All Rights Reserved. Permission to use, copy, modify, and distribute this
software and its documentation for educational, research, and not-for-profit
purposes, without fee and without a signed licensing agreement, is hereby
granted, provided that the above copyright notice, this paragraph and the
following two paragraphs appear in all copies, modifications, and
distributions. Contact The Office of Technology Licensing, UC Berkeley, 2150
Shattuck Avenue, Suite 510, Berkeley, CA 94720-1620, (510) 643-7201,
otl@berkeley.edu,
http://ipira.berkeley.edu/industry-info for commercial licensing opportunities.

IN NO EVENT SHALL REGENTS BE LIABLE TO ANY PARTY FOR DIRECT, INDIRECT, SPECIAL,
INCIDENTAL, OR CONSEQUENTIAL DAMAGES, INCLUDING LOST PROFITS, ARISING OUT OF
THE USE OF THIS SOFTWARE AND ITS DOCUMENTATION, EVEN IF REGENTS HAS BEEN
ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

REGENTS SPECIFICALLY DISCLAIMS ANY WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
PURPOSE. THE SOFTWARE AND ACCOMPANYING DOCUMENTATION, IF ANY, PROVIDED
HEREUNDER IS PROVIDED "AS IS". REGENTS HAS NO OBLIGATION TO PROVIDE
MAINTENANCE, SUPPORT, UPDATES, ENHANCEMENTS, OR MODIFICATIONS.

Displays robust grasps planned using a GQ-CNN-based policy on a set of saved
RGB-D images. The default configuration is cfg/examples/policy.yaml.

Author
------
Jeff Mahler
"""

import argparse
import logging
import numpy as np
import os
import rosgraph.roslogging as rl
import rospy
import sys

from cv_bridge import CvBridge, CvBridgeError

from autolab_core import Point, Logger
from perception import BinaryImage, CameraIntrinsics, ColorImage, DepthImage
from visualization import Visualizer2D as vis

from gqcnn.grasping import Grasp2D, SuctionPoint2D, GraspAction
from gqcnn.msg import GQCNNGrasp
from gqcnn.srv import GQCNNGraspPlanner, GQCNNGraspPlannerSegmask

# Set up time logger.
logger = Logger.get_logger("examples/policy_ros.py")


def run_gqcnn(f, depth_im, segmask, color_im, gripper_width, camera_intr_filename):
    namespace = "/gqcnn"
    
    # Enables GUI
    vis_grasp = False

    # Wait for grasp planning service and create service proxy.
    rospy.wait_for_service("%s/grasp_planner" % (namespace))
    rospy.wait_for_service("%s/grasp_planner_segmask" % (namespace))
    plan_grasp = rospy.ServiceProxy("%s/grasp_planner" % (namespace),
                                    GQCNNGraspPlanner)
    plan_grasp_segmask = rospy.ServiceProxy(
        "%s/grasp_planner_segmask" % (namespace), GQCNNGraspPlannerSegmask)
    cv_bridge = CvBridge()

    # Setup camera intrinsics.
    camera_intr = CameraIntrinsics.load(camera_intr_filename)

    # Assign frames to the intrinsic file frame
    depth_im.header.frame_id = camera_intr.frame
    color_im.header.frame_id = camera_intr.frame

    # Optional: Use segmask image, useful for multiple objects!
    # grasp_resp = plan_grasp_segmask(color_im, depth_im,
    # camera_intr.rosmsg, segmask)

    # Dont use seg image, only dealing with one object
    grasp_resp = plan_grasp(color_im, depth_im, camera_intr.rosmsg)

    grasp = grasp_resp.grasp

    # Convert to a grasp action.
    grasp_type = grasp.grasp_type
    if grasp_type == GQCNNGrasp.PARALLEL_JAW:
        center = Point(np.array([grasp.center_px[0], grasp.center_px[1]]),
                       frame=camera_intr.frame)
        grasp_2d = Grasp2D(center,
                           grasp.angle,
                           grasp.depth,
                           width=gripper_width,
                           camera_intr=camera_intr)
    elif grasp_type == GQCNNGrasp.SUCTION:
        center = Point(np.array([grasp.center_px[0], grasp.center_px[1]]),
                       frame=camera_intr.frame)
        grasp_2d = SuctionPoint2D(center,
                                  np.array([0, 0, 1]),
                                  grasp.depth,
                                  camera_intr=camera_intr)
    else:
        raise ValueError("Grasp type %d not recognized!" % (grasp_type))
    try:
        thumbnail = DepthImage(cv_bridge.imgmsg_to_cv2(
            grasp.thumbnail, desired_encoding="passthrough"),
            frame=camera_intr.frame)
    except CvBridgeError as e:
        logger.error(e)
        logger.error("Failed to convert image")
        sys.exit(1)
    action = GraspAction(grasp_2d, grasp.q_value, thumbnail)

    # Write time and q-value to a file (for NIST experiments)
    #f.write("\nq = " + str(action.q_value) + " ")

    # Visualize final grasp.
    # TODO: Currently outputs a blank image with the grasp, need to fix
    if vis_grasp:
        vis.figure(size=(10, 10))
        vis.imshow(segmask, vmin=0.6, vmax=0.9)
        vis.grasp(action.grasp, scale=2.5, show_center=False, show_axis=True)
        vis.title("Planned grasp on depth (Q=%.3f)" % (action.q_value))
        vis.show()
