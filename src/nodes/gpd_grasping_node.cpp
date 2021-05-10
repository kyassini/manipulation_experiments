/*****************************************************************
  Main Node for Gen3 GPD grasping
*****************************************************************/

#include "manipulation_class.hpp"
#include "perception_class.hpp"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "gpd_grasping_node");

  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(4);
  spinner.start();
  ros::Duration(1.0).sleep();

  std::string planning_group = "arm"; // Gen3 planning group

  Manipulation manipulation(nh, planning_group);
  Perception perception(nh);

  // Point cloud transform listener
  perception.transform_listener = TransformListenerPtr(
      new tf::TransformListener());

  // Manipulation variables/setup
  manipulation.planning_scene_ptr = PlanningScenePtr(
      new moveit::planning_interface::PlanningSceneInterface());
  manipulation.move_group_ptr = MoveGroupPtr(
      new moveit::planning_interface::MoveGroupInterface(manipulation.PLANNING_GROUP));

  manipulation.move_group_ptr->setPlanningTime(45.0);
  //manipulation.move_group_ptr->setMaxVelocityScalingFactor(0.5); // Changes arm speed
  manipulation.move_group_ptr->setPlannerId("RRT");

  while (ros::ok())
  {
    // Detach objects potentially left over from previous run
    manipulation.move_group_ptr->detachObject("object");
    manipulation.remove_objects();

    // Ensure gripper is open
    manipulation.open_gripper();

    // Take top snapshot, left and right angles also possible but not currently used
    manipulation.goTop();
    ros::Duration(5).sleep();
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
    manipulation.begin = ros::Time::now();

    // Set collision objects (grasping object, table)
    //manipulation.set_objects();

    // Wait to get grasps grom GPD
    while (manipulation.getting_grasps)
    {
    }

    // Main grasping ppanning code
    manipulation.path_planning();

    // Pickup the object
    if (manipulation.grabbed_object)
    {
      manipulation.pickup();
      //manipulation.place(0.25); // Place at inputed height offset
    }
    ros::Duration(1.0).sleep();

    // Test how well the arm has grasped the object
    manipulation.goPostGrasp();
    ros::Duration(3.0).sleep();

    // Go back to the top position for repeating, get a new grasp
    manipulation.goTop();
    manipulation.getting_grasps = true;
  }

  ros::waitForShutdown();

  return 0;
}