#include "manipulation_class.hpp"

Manipulation::Manipulation(ros::NodeHandle nodeHandle, std::string planning_group)
{
    PLANNING_GROUP = planning_group;
    grasp_config = nodeHandle.subscribe("/detect_grasps/clustered_grasps", 1, &Manipulation::callback, this);
    this->gripper_command = nodeHandle.advertise<control_msgs::GripperCommandActionGoal>("robotiq_2f_85_gripper_controller/gripper_cmd/goal", 10);
}

void Manipulation::getCurrentState()
{

    const robot_state::JointModelGroup *joint_model_group =
        move_group_ptr->getCurrentState()->getJointModelGroup(PLANNING_GROUP);
    current_state = move_group_ptr->getCurrentState();
    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);
}

void Manipulation::move(std::vector<double>)
{
    move_group_ptr->setJointValueTarget(joint_group_positions);
    move_group_ptr->move();
}

void Manipulation::setJointGroup(double j0, double j1, double j2, double j3, double j4, double j5, double j6)
{
    joint_group_positions[0] = j0; // base
    joint_group_positions[1] = j1;
    joint_group_positions[2] = j2;
    joint_group_positions[3] = j3;
    joint_group_positions[4] = j4;
    joint_group_positions[5] = j5;
    joint_group_positions[6] = j6; // gripper
}

/*
void Manipulation::goTop()
{
    getCurrentState();
    ROS_INFO("Moving to Top Position");
    setJointGroup(0, pi / 4, 0, pi / 4, 0, pi / 1.75, -pi / 2);
    move(joint_group_positions);
}
*/
void Manipulation::goRight()
{
    getCurrentState();
    ROS_INFO("Moving to Right Position");
    setJointGroup(pi / 1.8, pi / 3, -pi / 3, pi / 2, 0, pi / 3, pi / 7);
    move(joint_group_positions);
}

void Manipulation::goLeft()
{
    getCurrentState();
    ROS_INFO("Moving to Left Position");
    setJointGroup(-pi / 1.8, pi / 3, pi / 3, pi / 2, 0, pi / 3, 2.8);
    move(joint_group_positions);
}

void Manipulation::goVertical()
{
    getCurrentState();
    ROS_INFO("Moving to Vertical Position");
    setJointGroup(0, 0, 0, 0, 0, 0, 0);
    move(joint_group_positions);
}

void Manipulation::goTop()
{
    getCurrentState();
    ROS_INFO("Moving to Top Position");
    setJointGroup(0, pi / 4, 0, pi / 4, 0, pi / 2, -pi / 2);
    move(joint_group_positions);
}
/*
void Manipulation::goRight()
{
    getCurrentState();
    ROS_INFO("Moving to Right Position");
    setJointGroup(pi / 2, pi / 2, -pi / 3, pi / 2, 0, pi / 4, pi / 7);
    move(joint_group_positions);
}

void Manipulation::goLeft()
{
    getCurrentState();
    ROS_INFO("Moving to Left Position");
    setJointGroup(-pi / 2, pi / 2, pi / 3, pi / 2, 0, pi / 4, 2.8);
    move(joint_group_positions);
}
*/
void Manipulation::goWait()
{
    getCurrentState();
    ROS_INFO("Moving to Waiting Position");
    setJointGroup(0, pi / 5, 0, pi / 2, 0, -pi / 5, -pi / 2);
    move(joint_group_positions);
}

void Manipulation::set_objects()
{
    std::vector<moveit_msgs::CollisionObject> collision_objects;
    collision_objects.resize(1);

    collision_objects[0].id = "object";
    collision_objects[0].header.frame_id = "base_link"; //world

    // Define the primitive and its dimensions
    collision_objects[0].primitives.resize(1);
    collision_objects[0].primitives[0].type = collision_objects[0].primitives[0].CYLINDER;
    collision_objects[0].primitives[0].dimensions.resize(3);
    collision_objects[0].primitives[0].dimensions[0] = 0.01; //h
    collision_objects[0].primitives[0].dimensions[1] = 0.01; //r

    // Define the pose of the object
    collision_objects[0].primitive_poses.resize(1);
    collision_objects[0].primitive_poses[0].position.x = this->pose.x;
    collision_objects[0].primitive_poses[0].position.y = this->pose.y;
    collision_objects[0].primitive_poses[0].position.z = this->pose.z;

    collision_objects[0].operation = collision_objects[0].ADD;
    this->planning_scene_ptr->applyCollisionObjects(collision_objects);
    this->collision_object = collision_objects[0];
}

void Manipulation::remove_objects()
{
    std::vector<std::string> object_ids;
    object_ids.push_back("object");
    this->planning_scene_ptr->removeCollisionObjects(object_ids);
}

void Manipulation::closedGripper(trajectory_msgs::JointTrajectory &posture)
{
    //this->gripper_cmd.goal.command.position = 0.5;
    //gripper_command.publish(gripper_cmd);
    //ROS_WARN("CLOSE GRIPPER");
    //Add finger joints
    posture.joint_names.resize(6);
    posture.joint_names[0] = "left_inner_finger_joint";
    posture.joint_names[1] = "right_inner_knuckle_joint";
    posture.joint_names[2] = "finger_joint";
    posture.joint_names[3] = "right_inner_finger_joint";
    posture.joint_names[4] = "left_inner_knuckle_joint";
    posture.joint_names[5] = "right_outer_knuckle_joint";


    // Set them as closed.
    posture.points.resize(1);
    posture.points[0].positions.resize(6);
    posture.points[0].positions[0] = 0.8;
    posture.points[0].positions[1] = 0.8;
    posture.points[0].positions[2] = 0.8;
    posture.points[0].positions[3] = -0.8;
    posture.points[0].positions[4] = 0.8;
    posture.points[0].positions[5] = 0.8;
    posture.points[0].time_from_start = ros::Duration(0.5);
}

void Manipulation::openGripper(trajectory_msgs::JointTrajectory &posture)
{
    //this->gripper_cmd.goal.command.position = 0;
    //gripper_command.publish(gripper_cmd);
    //ROS_WARN("OPEN GRIPPER");

    //Add finger joints
    posture.joint_names.resize(6);
    posture.joint_names[0] = "left_inner_finger_joint";
    posture.joint_names[1] = "right_inner_knuckle_joint";
    posture.joint_names[2] = "finger_joint";
    posture.joint_names[3] = "right_inner_finger_joint";
    posture.joint_names[4] = "left_inner_knuckle_joint";
    posture.joint_names[5] = "right_outer_knuckle_joint";

    // Set them as open, wide enough for the object to fit.
    posture.points.resize(1);
    posture.points[0].positions.resize(6);
    posture.points[0].positions[0] = 0;
    posture.points[0].positions[1] = 0;
    posture.points[0].positions[2] = 0;
    posture.points[0].positions[3] = 0;
    posture.points[0].positions[4] = 0;
    posture.points[0].positions[5] = 0;

    posture.points[0].time_from_start = ros::Duration(0.5);
}