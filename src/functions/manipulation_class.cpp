#include "manipulation_class.hpp"

Manipulation::Manipulation(ros::NodeHandle nodeHandle, std::string planning_group)
{
  PLANNING_GROUP = planning_group;
  move_group_ptr = moveit::planning_interface::MoveGroupInterfacePtr(
      new moveit::planning_interface::MoveGroupInterface(PLANNING_GROUP));
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
  moveit::planning_interface::MoveGroupInterface move_group_ptr(PLANNING_GROUP);
  move_group_ptr.setJointValueTarget(joint_group_positions);
  move_group_ptr.move();
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

void Manipulation::goTop()
{
  getCurrentState();
  ROS_INFO("Moving to Top Position");
  setJointGroup(0, pi / 4, 0, pi / 4, 0, pi / 2, -pi / 2);
  move(joint_group_positions);
}

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

void Manipulation::goVertical()
{
  getCurrentState();
  ROS_INFO("Moving to Vertical Position");
  setJointGroup(0, 0, 0, 0, 0, 0, 0);
  move(joint_group_positions);
}
