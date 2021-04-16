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

from gqcnn_ros import GQCNN_imgs

def callback(msg):
    rospy.logwarn("Grasp msg received...")
    print(msg)

    """
    listener = tf.TransformListener()
    listener.waitForTransform("/base_link", "/camera_depth_frame", rospy.Time(), rospy.Duration(4.0))
    try:
        now = rospy.Time.now()
        listener.waitForTransform("/base_link", "/camera_depth_frame", now, rospy.Duration(4.0))
        (trans,rot) = listener.lookupTransform("/base_link", "/camera_depth_frame", now)
        pose_transformed = listener.transformPose('/base_link', msg)

    except (tf.LookupException, tf.ConnectivityException):
        rospy.signal_shutdown()
    """
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

    pose_transformed.pose.position.z+=0.02

    print(pose_transformed)

    group.set_pose_target(pose_transformed)
    plan = group.go(wait=True)
    group.stop()
    group.clear_pose_targets()

    rospy.sleep(2)
    go_to_start(group)

def go_to_start(group):
    joint_goal = group.get_current_joint_values()
    joint_goal[0] = 0
    joint_goal[1] = pi / 6
    joint_goal[2] = 0
    joint_goal[3] = pi / 4
    joint_goal[4] = 0
    joint_goal[5] = pi / 1.75
    joint_goal[6] = -pi / 2

    rospy.logwarn("Moving to starting position...")
    group.go(joint_goal, wait=True)
    group.stop()

if __name__ == "__main__":
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('gen3_moveit',
                    anonymous=True)
    rospy.Subscriber("/gqcnn_grasp/pose", PoseStamped, callback)

    imgs = GQCNN_imgs()
    
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()

    group_name = "arm"
    group = moveit_commander.MoveGroupCommander(group_name)

    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)

    planning_frame = group.get_planning_frame()
    eef_link = group.get_end_effector_link()
    group_names = robot.get_group_names()

    go_to_start(group)

    rospy.logwarn("Waiting for grasp msg...")
    rospy.sleep(2)
    imgs.save_imgs()

    rospy.spin()

    """
    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.orientation.z = 1
    pose_goal.position.x = 0.4
    pose_goal.position.y = 0.1
    pose_goal.position.z = 0.4
    group.set_pose_target(pose_goal)

    plan = group.go(wait=True)
    group.stop()
    group.clear_pose_targets()
    """
