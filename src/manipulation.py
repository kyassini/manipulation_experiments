#!/usr/bin/env python

import sys
import tf
import tf2_ros
import tf2_geometry_msgs

import rospy
import moveit_commander
import moveit_msgs.msg
from geometry_msgs.msg import PoseStamped
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list


class gen3_movegroup:
    def __init__(self, moveit_commander):
        rospy.Subscriber("/gqcnn_grasp/pose", PoseStamped, self.callback)

        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()

        self.group_name = "arm"
        
        self.group = moveit_commander.MoveGroupCommander(self.group_name)

        self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                            moveit_msgs.msg.DisplayTrajectory,
                                                            queue_size=20)

        self.planning_frame = self.group.get_planning_frame()
        self.eef_link = self.group.get_end_effector_link()
        self.group_names = self.robot.get_group_names()

    def go_to_start(self):
        joint_goal = self.group.get_current_joint_values()

        joint_goal[0] = 0
        joint_goal[1] = pi / 6
        joint_goal[2] = 0
        joint_goal[3] = pi / 4
        joint_goal[4] = 0
        joint_goal[5] = pi / 1.75
        joint_goal[6] = -pi / 2

        rospy.logwarn("Moving to starting position...")
        self.group.go(joint_goal, wait=True)
        self.group.stop()

    def callback(self, msg):
        rospy.logwarn("Grasp msg received...")

        tfBuffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(tfBuffer)

        try:
            trans = tfBuffer.lookup_transform(
                'world', msg.header.frame_id, rospy.Time(0), rospy.Duration(10))
            pose_transformed = tf2_geometry_msgs.do_transform_pose(msg, trans)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logerr("No frame found!")

        quaternion = tf.transformations.quaternion_from_euler(-pi, 0, 0)
        pose_transformed.pose.orientation.x = quaternion[0]
        pose_transformed.pose.orientation.y = quaternion[1]
        pose_transformed.pose.orientation.z = quaternion[2]
        pose_transformed.pose.orientation.w = quaternion[3]

        hand_offset = 0.06
        pre_grasp = 0.12

        pose_transformed.pose.position.z += pre_grasp
        rospy.logwarn("Moving to pre-grasp")
        self.group.set_pose_target(pose_transformed)
        plan = self.group.go(wait=True)
        self.group.stop()
        self.group.clear_pose_targets()

        pose_transformed.pose.position.z -= (pre_grasp - hand_offset)
        rospy.logwarn("Moving to grasp")
        self.group.set_pose_target(pose_transformed)
        plan = self.group.go(wait=True)
        self.group.stop()
        self.group.clear_pose_targets()
