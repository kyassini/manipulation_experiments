#ifndef MANIPULATION_CLASS_HPP
#define MANIPULATION_CLASS_HPP

#include <ros/ros.h>
#include <boost/filesystem.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit_msgs/PickupAction.h>

#include "gpd_ros/GraspConfigList.h"
#include "gpd_ros/GraspConfig.h"

#include <std_msgs/Float32.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <control_msgs/GripperCommandActionGoal.h>

typedef boost::shared_ptr<moveit::planning_interface::MoveGroupInterface> MoveGroupPtr;
typedef boost::shared_ptr<moveit::planning_interface::PlanningSceneInterface> PlanningScenePtr;

class Manipulation
{
private:
  ros::Subscriber grasp_config;

  const double pi = std::acos(-1); // Create pi constant (3.14..)

  void setJointGroup(double j0, double j1, double j2,
                     double j3, double j4, double j5, double j6);

  void getCurrentState();
  void move(std::vector<double>);

public:
  ros::Publisher gripper_command;

  Manipulation(ros::NodeHandle nodeHandle, std::string planning_group);

  moveit::core::RobotStatePtr current_state;
  std::vector<double> joint_group_positions;
  std::string PLANNING_GROUP;
  PlanningScenePtr planning_scene_ptr;
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  MoveGroupPtr move_group_ptr;
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  void goTop();
  void goRight();
  void goLeft();
  void goVertical();
  void goWait();
  void set_objects();
  void remove_objects();

  void closedGripper(trajectory_msgs::JointTrajectory &);
  void openGripper(trajectory_msgs::JointTrajectory &);
  control_msgs::GripperCommandActionGoal gripper_cmd;

  // GPD
  void callback(const gpd_ros::GraspConfigList msg);
  void path_planning();
  void set_target_pose();
  void plan_pose_goal();
  void pick_and_place();

  bool getting_grasps = true;
  bool pose_success;

  gpd_ros::GraspConfigList grasp_candidates;
  gpd_ros::GraspConfig grasp;
  std_msgs::Float32 score;
  bool grabbed_object;

  geometry_msgs::Pose target_pose;
  geometry_msgs::Vector3 orientation;

  geometry_msgs::Point pose;
  geometry_msgs::Point pose_sample;
  geometry_msgs::Vector3 grasp_orientation;
  moveit_msgs::CollisionObject collision_object;

  tf2::Quaternion q;
};

#endif