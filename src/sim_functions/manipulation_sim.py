############################################################
# Manipulation Class - Using the grasp pose from GQCNN,    #
#   grasp the object using the sim arm                     # 
############################################################

import sys
import tf
import tf2_ros
import tf2_geometry_msgs

import rospy
import moveit_commander
import moveit_msgs.msg
from moveit_commander.conversions import pose_to_list

from geometry_msgs.msg import PoseStamped
from control_msgs.msg import GripperCommandActionGoal
from std_msgs.msg import String

from math import pi


class gen3_movegroup:
    def __init__(self, moveit_commander):
        rospy.Subscriber("/gqcnn_grasp/pose", PoseStamped, self.callback) # Output of gqcnn
        self.gripper_cmd_pub = rospy.Publisher(
            "robotiq_2f_85_gripper_controller/gripper_cmd/goal", GripperCommandActionGoal) # Control gripper
        self.gripper_cmd = GripperCommandActionGoal()

        ### Gen3 Moveit setup ###
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

        self.msg_recieved = False
        
        # Open file for logging times and score
        self.f = None #open("/home/kyassini/Desktop/Testing_Logs/test_gqcnn.txt", "a")

    # Get current time for logging
    def update_starttime(self):
        self.start = rospy.get_time()

    """ Gripper commands """
    def close_gripper(self):
        self.gripper_cmd.goal.command.position = 0.65
        self.gripper_cmd_pub.publish(self.gripper_cmd)
        rospy.logwarn("Closing Gripper...")

    def open_gripper(self):
        self.gripper_cmd.goal.command.position = 0
        self.gripper_cmd_pub.publish(self.gripper_cmd)
        rospy.logwarn("Opening Gripper...")

    """ Arm commands """
    def go_to_start(self):
        joint_goal = self.group.get_current_joint_values()

        joint_goal[0] = -0.15524737784951537
        joint_goal[1] = 0.22333506921426916
        joint_goal[2] = 0.17610458971674323
        joint_goal[3] = 1.3338326472154447
        joint_goal[4] = -0.03207457953669568
        joint_goal[5] = 1.581784662610618
        joint_goal[6] = -1.5642204494312733


        rospy.logwarn("Moving to starting position...")
        self.group.go(joint_goal, wait=True)
        self.group.stop()

    def go_to_post_grasp(self):
        joint_goal = self.group.get_current_joint_values()

        joint_goal[0] = -0.06
        joint_goal[1] = 0.4309
        joint_goal[2] = 0.0458
        joint_goal[3] = 2.1915
        joint_goal[4] = -0.0325
        joint_goal[5] = -1.043
        joint_goal[6] = -1.567

        rospy.logwarn("Moving to Post Grasp position...")
        self.group.go(joint_goal, wait=True)
        self.group.stop()


    # Callback from gqcnn_grasp/pose topic
    def callback(self, msg):
        rospy.logwarn("Grasp msg received...")
        self.msg_recieved = True


        """ Transfrom pose from the header frame it comes from to base_link
                frame depends on whether sim or real arm is used
        """
        tfBuffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(tfBuffer)

        try:
            trans = tfBuffer.lookup_transform(
                'base_link', msg.header.frame_id, rospy.Time(0), rospy.Duration(10))
            pose_transformed = tf2_geometry_msgs.do_transform_pose(msg, trans)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logerr("No frame found!")


        """
            Pose from gqcnn has a different axis due to a different gripper being used on their end
            Attempt at the proper transfrom is in the maniuplation.py for the real arm.
            For here, the orientation is locked at perpendicular to the floor since that is what it's supposed to be anyway 
        """
        quaternion = tf.transformations.quaternion_from_euler(-pi, 0, 0)
        pose_transformed.pose.orientation.x = quaternion[0]
        pose_transformed.pose.orientation.y = quaternion[1]
        pose_transformed.pose.orientation.z = quaternion[2]
        pose_transformed.pose.orientation.w = quaternion[3]


        # Offset values
        hand_offset = 0.08
        pre_grasp = 0.12

        # Apply pre-grasp offset to the pose
        pose_transformed.pose.position.z += pre_grasp

        """ Pre-grasp """
        rospy.logwarn("Moving to pre-grasp")
        self.group.set_pose_target(pose_transformed)
        plan = self.group.go(wait=True)
        self.group.stop()
        self.group.clear_pose_targets()

        """ Grasp """
        pose_transformed.pose.position.z -= hand_offset
        rospy.logwarn("Moving to grasp")
        self.group.set_pose_target(pose_transformed)
        plan = self.group.go(wait=True)
        self.group.stop()
        self.group.clear_pose_targets()

        self.close_gripper()

        """ Post-grasp """
        pose_transformed.pose.position.z += pre_grasp
        rospy.logwarn("Moving away from grasp")
        self.group.set_pose_target(pose_transformed)
        plan = self.group.go(wait=True)
        self.group.stop()
        self.group.clear_pose_targets()

        """ Hold the grasp, and then go back to start """
        self.go_to_post_grasp()
        self.go_to_start()

        rospy.sleep(1)
        self.open_gripper()