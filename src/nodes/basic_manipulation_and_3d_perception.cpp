#include "manipulation_class.hpp"
#include "perception_class.hpp"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "basic_manipulation_and_3d_perception_node");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(4);
  spinner.start();
  ros::Duration(1.0).sleep();

  std::string planning_group = "arm";

  Manipulation manipulation(nh, planning_group);
  Perception perception(nh);

  perception.transform_listener = TransformListenerPtr(
      new tf::TransformListener());

  manipulation.planning_scene_ptr = PlanningScenePtr(
      new moveit::planning_interface::PlanningSceneInterface());
  manipulation.move_group_ptr = MoveGroupPtr(
      new moveit::planning_interface::MoveGroupInterface(manipulation.PLANNING_GROUP));

  manipulation.move_group_ptr->detachObject("object");
  manipulation.remove_objects();

  manipulation.move_group_ptr->setPlanningTime(45.0);
  manipulation.move_group_ptr->setMaxVelocityScalingFactor(0.50);
  manipulation.move_group_ptr->setPlannerId("SBL");

  // open gripper
  manipulation.gripper_cmd.goal.command.position = 0;
  manipulation.gripper_command.publish(manipulation.gripper_cmd);


  //manipulation.open_gripper();

  while (ros::ok())
  {
    manipulation.goTop();
    ros::Duration(3).sleep();
    perception.snapshot_top();
    ros::Duration(1).sleep();
    /*
    manipulation.goRight();
    ros::Duration(3).sleep();
    perception.snapshot_right();
    ros::Duration(1).sleep();

    manipulation.goLeft();
    ros::Duration(3).sleep();
    perception.snapshot_left();
    ros::Duration(1).sleep();
    */
    perception.concatenate_clouds();

    manipulation.goTop();
    ros::Duration(5).sleep();

    while (manipulation.getting_grasps)
    {
    }

    manipulation.path_planning();
    if (manipulation.grabbed_object)
    {
      manipulation.set_objects();
      manipulation.pick_and_place();

      //manipulation.close_gripper();
    }
    ros::Duration(5.0).sleep();
    ros::shutdown();
  }
  ros::waitForShutdown();

  return 0;
}