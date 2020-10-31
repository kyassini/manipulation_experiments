#include "manipulation_class.hpp"

Manipulation::Manipulation(ros::NodeHandle nodeHandle, std::string planning_group)
{
  PLANNING_GROUP = planning_group;
  move_group_ptr = moveit::planning_interface::MoveGroupInterfacePtr(
      new moveit::planning_interface::MoveGroupInterface(PLANNING_GROUP));
}

void Manipulation::goRight()
{
  getCurrentState();

  joint_group_positions[0] = 0;
  joint_group_positions[1] = 3.14 / 4;
  joint_group_positions[2] = 0;
  joint_group_positions[3] = 3.14 / 4;
  joint_group_positions[4] = 3.14 / 4;
  joint_group_positions[5] = 3.14 / 4;
  joint_group_positions[6] = -3.14 / 2;

  move();
}

void Manipulation::getCurrentState()
{

  const robot_state::JointModelGroup *joint_model_group =
      move_group_ptr->getCurrentState()->getJointModelGroup(PLANNING_GROUP);
  current_state = move_group_ptr->getCurrentState();
  current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);
}

void Manipulation::move()
{
  moveit::planning_interface::MoveGroupInterface move_group_ptr(PLANNING_GROUP);
  move_group_ptr.setJointValueTarget(joint_group_positions);
  move_group_ptr.move();
}
