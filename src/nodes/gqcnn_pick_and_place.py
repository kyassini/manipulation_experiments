#!/usr/bin/env python
from manipulation import *
from camera import *
from policy_ros import run_gqcnn


if __name__ == "__main__":
    rospy.init_node('gqcnn_pick_and_place',
                    anonymous=True)

    # Setup moveit group
    moveit_commander.roscpp_initialize(sys.argv)
    arm = gen3_movegroup(moveit_commander)

    # Camera subscribers
    cams = Camera()

    arm.go_to_start() # Starting arm position
    rospy.sleep(5)

    # Get ros msgs for depth img, segmask, and color img 
    while True:
        try:
            depth, seg, color = cams.get_imgs()
            break
        except:
            rospy.logerr("Ca ..")
    
    
    rospy.sleep(1)

    # Run grasp planner
    run_gqcnn(depth, seg, color, 0.085, "/home/kyassini/catkin_ws/src/gqcnn_pick_and_place/gen3.intr")

    rospy.spin()
