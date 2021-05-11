#!/usr/bin/env python

################################################### 
# GQCNN grasping node - main driver               #
###################################################


import rospy
import sys

# Check launch arg for loading sim or real arm code.
# Should be able to do this in a better way...

sim = rospy.myargv(argv=sys.argv)

if sim[1] == "true":
    print("Loading sim config")
    from sim_functions.manipulation_sim import *
    from sim_functions.camera_sim import *
    intr = "/home/kyassini/catkin_ws/src/gqcnn_pick_and_place/gen3.intr"
else:
    print("Loading Real arm config")
    from real_arm_functions.manipulation import *
    from real_arm_functions.camera import *
    intr = "/home/kyassini/catkin_ws/src/gqcnn_pick_and_place/gen3_real.intr"

from policy_ros import run_gqcnn

if __name__ == "__main__":
    rospy.init_node('gqcnn_pick_and_place',
                    anonymous=True)

    # Setup moveit group for the arm
    moveit_commander.roscpp_initialize(sys.argv)
    arm = gen3_movegroup(moveit_commander)

    # Camera subscribers
    cams = Camera()

    arm.open_gripper() # Open gripper at start
   
    arm.go_to_start() # Starting arm position
    
    rospy.sleep(2)

    # Get depth numpy array, binary segmented image, and the color image from the wrist camera
    depth, seg, color = cams.get_imgs()

    rospy.sleep(1)

    # Run grasp planner service
    arm.update_starttime()

    # Send times file (currently disabled), depth array, seg image, color image, gripper width, and intrinsic calib file
    #   to gqcnn node
    run_gqcnn(arm.f, depth, seg, color, 0.085, intr)
    
    rospy.spin()