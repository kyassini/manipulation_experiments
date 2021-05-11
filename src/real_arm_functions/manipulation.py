############################################################
# Manipulation Class - Using the grasp pose from GQCNN,    #
#   grasp the object using the real arm                    # 
############################################################

import sys
import numpy as np
import rospy

import tf
import tf2_ros
import tf2_geometry_msgs
from tf import TransformerROS
from tf.transformations import quaternion_matrix

import moveit_commander
import moveit_msgs.msg
from moveit_commander.conversions import pose_to_list

from math import pi
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, Quaternion
from control_msgs.msg import GripperCommandActionGoal


class gen3_movegroup:
    def __init__(self, moveit_commander):
        rospy.Subscriber("/gqcnn_grasp/pose", PoseStamped, self.callback) # GQCNN action output
        
        """ Gripper control """
        self.gripper_cmd_pub = rospy.Publisher(
            "robotiq_2f_85_gripper_controller/gripper_cmd/goal", GripperCommandActionGoal)
        self.gripper_cmd = GripperCommandActionGoal()
        
        self.final_pose_pub = rospy.Publisher('/final_pose', PoseStamped, queue_size=1) # Visualize pose on rviz

        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()

        self.group_name = "arm" # gen3 group

        self.group = moveit_commander.MoveGroupCommander(self.group_name)

        self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                            moveit_msgs.msg.DisplayTrajectory,
                                                            queue_size=20)

        self.planning_frame = self.group.get_planning_frame()
        self.eef_link = self.group.get_end_effector_link()
        self.group_names = self.robot.get_group_names()

        self.msg_recieved = False
        self.start = 0
        self.end = 0

        # Log file
        self.f = None #open("/home/kyassini/Desktop/Testing_Logs/test_gqcnn.txt", "a")

    # Get current time
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
        rospy.logerr(msg)

        """
         Note: the following code is in an experimental state...
         Have not been able to get a "proper" method for transforming the gqcnn pose to 
         a pose with the correct orrientation for the arm to use. This is due to their gripper having differnt axes then ours...
         See this github issue for a similar problem: https://github.com/BerkeleyAutomation/gqcnn/issues/112
        
         Currently, it just transforms from camera frame to base_link which results in the correct
            position, then locks the orietnation to perpendicular while rotating the gripper by the published x value
            from the publsihed quarternion
        """

        tfBuffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(tfBuffer)

        try:
            trans = tfBuffer.lookup_transform(
                'base_link', msg.header.frame_id, rospy.Time(0), rospy.Duration(10))
            rospy.logerr(trans)

            """
                Below are my attempts at right shifting the roation matrix so that the axes from the 
                gqcnn pose align with the gripper.
                On rviz, if you look at the axes of the transformed pose and the axes of the gripper you will see that the axes are swapped.   
            """

            """
            tf_r = tf.TransformerROS()
            matrix = tf_r.fromTranslationRotation((trans.transform.translation.x,
                                                   trans.transform.translation.y,
                                                   trans.transform.translation.z),
                                                  (trans.transform.rotation.x,
                                                   trans.transform.rotation.y,
                                                   trans.transform.rotation.z,
                                                   trans.transform.rotation.w))

            # matrix = matrix * np.array([[0], [0], [-1]])
            print(matrix)

            rot_matrix = matrix[np.ix_([0, 1, 2], [0, 1, 2])]
            print(rot_matrix)

            matrix[np.ix_([0, 1, 2], [0, 1, 2])] = np.matmul(matrix[np.ix_(
              [0, 1, 2], [0, 1, 2])], np.array([[0, 0, 1], [-1, 0, 0], [0, 0, 1]]))
            #matrix[np.ix_([0, 1, 2], [0, 1, 2])] *= np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]])

            #tets = tf.transformations.quaternion_matrix((1,0,0,0))
            #matrix = np.matmul(matrix, tets)

            print(matrix)

            translation = tf.transformations.translation_from_matrix(matrix)
            rotation = tf.transformations.quaternion_from_matrix(matrix)
            # rot = tf.transformations.
            # print(rot)
            print(translation)
            print(rotation)

            trans.transform.translation.x = translation[0]
            trans.transform.translation.y = translation[1]
            trans.transform.translation.z = translation[2]

            trans.transform.rotation.x = 1 * rotation[0]
            trans.transform.rotation.y = 1 * rotation[1]
            trans.transform.rotation.z = 1 * rotation[2]
            trans.transform.rotation.w = 1 * rotation[3]
            
            
            rospy.logerr(trans)

            
            qw = trans.transform.rotation.w
            qx = trans.transform.rotation.x
            qy = trans.transform.rotation.y
            qz = trans.transform.rotation.z

            rot_matrix = np.array([[1-2*qy**2-2*qz**2, 2*qx*qy-2*qz*qw, 2*qx*qz+2*qy*qw],
                                    [2*qx*qy+2*qz*qw, 1-2*qx**2- \
                                        2*qz**2, 2*qy*qz-2*qx*qw],
                                    [2*qx*qz-2*qy*qw, 2*qy*qz+2*qx*qw, 1-2*qx**2-2*qy**2]])

            shift = np.array([[0,0,1],[1,0,0],[0,1,0]])
            # rospy.logwarn(shift)

            new = np.dot(rot_matrix, shift)
            # rospy.logwarn(new)

            new = np.vstack((new, np.array([0,0,0])))
            new = np.append(new, [[0],[0],[0], [1]], axis= 1)
            rospy.logwarn(new)

            # rospy.logwarn(new)
            quart = tf.transformations.quaternion_from_matrix(new)
            trans.transform.rotation.x = quart[0]
            trans.transform.rotation.y = quart[1]
            trans.transform.rotation.z = quart[2]
            trans.transform.rotation.w = quart[3]
            """

            pose_transformed = tf2_geometry_msgs.do_transform_pose(msg, trans)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logerr("No frame found!")

        """
        # rospy.logerr(trans.transform.rotation)

        # rot_matrix = m(trans.transform.rotation)
        #         #rospy.logwarn(matrix)

        #
        # rospy.logwarn(shift)
        # rospy.logwarn(matrix)

        # m = matrix * shift

        # pose_transformed.pose.orientation.x = quart[0]
        # pose_transformed.pose.orientation.y = quart[1]
        # pose_transformed.pose.orientation.z = quart[2]
        # pose_transformed.pose.orientation.w = quart[3]

        # ete = tf.transformations.euler_from_quaternion(
            # (pose_transformed.pose.orientation.x, pose_transformed.pose.orientation.y, pose_transformed.pose.orientation.z, pose_transformed.pose.orientation.w))
        # print(ete)
        """
        qx = pose_transformed.pose.orientation.x 
        qy = pose_transformed.pose.orientation.y
        qz = pose_transformed.pose.orientation.z
        qw = pose_transformed.pose.orientation.w


        quaternion = tf.transformations.quaternion_from_euler(-pi, 0, qx)
    # quaternion = tf.transformations.quaternion_from_euler(-pi, 0, ete[0])
        pose_transformed.pose.orientation.x = quaternion[0]
        pose_transformed.pose.orientation.y = quaternion[1]
        pose_transformed.pose.orientation.z = quaternion[2]
        pose_transformed.pose.orientation.w = quaternion[3]

        #pose_transformed.pose.orientation.x = -1
        #pose_transformed.pose.orientation.y = 0
        #pose_transformed.pose.orientation.z = 0
        #pose_transformed.pose.orientation.w = 0
        
        #pose_transformed.pose.position.x = trans.transform.translation.x
        #pose_transformed.pose.position.y = trans.transform.translation.y
        
        # Offset values
        hand_offset = 0.08  # 0.06
        pre_grasp = 0.17

        # Apply pre-grasp offset to the pose
        pose_transformed.pose.position.z += 0.14  # 0.10
        rospy.logerr(pose_transformed)

        self.final_pose_pub.publish(pose_transformed) # Show the transformed pose on RVIZ


        """ Pre-grasp """
        rospy.logwarn("Moving to pre-grasp")
        self.group.set_pose_target(pose_transformed)
        plan = self.group.go(wait=True)
        self.group.stop()
        self.group.clear_pose_targets()
        
        """ Grasp """
        pose_transformed.pose.position.z -= 0.07
        rospy.logwarn("Moving to grasp")
        self.group.set_pose_target(pose_transformed)
        plan = self.group.go(wait=True)
        self.group.stop()
        self.group.clear_pose_targets()

        """ For time logging purposes """
        #self.end = rospy.get_time()
        #self.f.write(",t = "+ str(self.end - self.start))

        self.close_gripper()

        """ Post-grasp """
        pose_transformed.pose.position.z += 0.14
        rospy.logwarn("Moving away from grasp")
        self.group.set_pose_target(pose_transformed)
        plan = self.group.go(wait=True)
        self.group.stop()
        self.group.clear_pose_targets()

        """ Hold the grasp, and then go back to start """

        self.go_to_post_grasp()
        rospy.sleep(3)
        self.go_to_start()

        self.open_gripper()
        self.f.close()
        
        rospy.signal_shutdown("End")
