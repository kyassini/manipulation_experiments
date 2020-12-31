#include "manipulation_class.hpp"
#include "perception_class.hpp"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "basic_manipulation_and_3d_perception_node");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);
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

  // Set useful variables before robot manipulation begins
  /*
  manipulation.move_group_ptr->setPlanningTime(45.0);
  manipulation.move_group_ptr->setMaxVelocityScalingFactor(0.25);
  manipulation.move_group_ptr->setPoseReferenceFrame("world");
  manipulation.move_group_ptr->setPlannerId("RRTConnect");
  */
  manipulation.open_gripper();

  while (ros::ok())
  {
    //Testing manipulation class...

    manipulation.goTop();
    ros::Duration(1).sleep();
    perception.snapshot_top();
    ros::Duration(1).sleep();

    manipulation.goRight();
    ros::Duration(1).sleep();
    perception.snapshot_right();
    ros::Duration(1).sleep();

    manipulation.goLeft();
    ros::Duration(1).sleep();
    perception.snapshot_left();
    ros::Duration(1).sleep();

    perception.concatenate_clouds();
    manipulation.goTop();

    while (manipulation.getting_grasps)
    {
    }

    manipulation.path_planning();
    if (manipulation.grabbed_object)
    {
      manipulation.set_objects();

      manipulation.pick_and_place();

      manipulation.close_gripper();
    }
  }
  ros::waitForShutdown();

  return 0;
}