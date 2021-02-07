/*****************************************************************
  Main Node for Gen3 Pick_and_Place
  -Kevin Yassini (kevin_yassini@student.uml.edu)
*****************************************************************/

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

    manipulation.move_group_ptr->setPlanningTime(45.0);
    manipulation.move_group_ptr->setMaxVelocityScalingFactor(0.5);
    manipulation.move_group_ptr->setPlannerId("SBL");

  while (ros::ok())
  {
    // Detach objects potentially left over from previous run
    manipulation.move_group_ptr->detachObject("object");
    manipulation.remove_objects();

    // Ensure gripper is open
    manipulation.open_gripper();

    // Take top snapshot, left and right angles also possible
    manipulation.goTop();
    ros::Duration(1).sleep();
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

    // Combine clouds and publish, for now only one cloud is used (top)
    perception.concatenate_clouds();

    // Set collision objects (grasping object, table)
    manipulation.set_objects();

    // Get grasps grom GPD
    while (manipulation.getting_grasps)
    {
    }

    // Main grasping code
    manipulation.path_planning();
    if (manipulation.grabbed_object)
    {
      manipulation.pickup();
      manipulation.place(0.12); // Place at inputed height offset
    }
    ros::Duration(1.0).sleep();

    // Go back to top position for repeating
    manipulation.goTop();
  }

  ros::waitForShutdown();

  return 0;
}