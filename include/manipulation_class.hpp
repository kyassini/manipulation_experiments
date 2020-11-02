#ifndef MANIPULATION_CLASS_HPP
#define MANIPULATION_CLASS_HPP

#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

class Manipulation
{
private:
    moveit::core::RobotStatePtr current_state;
    std::vector<double> joint_group_positions;
    std::string PLANNING_GROUP;
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    moveit::planning_interface::MoveGroupInterfacePtr move_group_ptr;

    const double pi = std::acos(-1); // Create pi constant (3.14..)
    
    void setJointGroup(double j0, double j1, double j2,
                       double j3, double j4, double j5, double j6);

    void getCurrentState();
    void move(std::vector<double>);

public:
    Manipulation(ros::NodeHandle nodeHandle, std::string planning_group);

    void goTop();
    void goRight();
    void goLeft();
    void goVertical();
};

#endif