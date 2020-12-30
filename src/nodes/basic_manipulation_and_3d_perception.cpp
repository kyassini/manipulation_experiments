#include "manipulation_class.hpp"
#include "perception_class.hpp"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "basic_manipulation_and_3d_perception_node");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  std::string planning_group = "arm";

  Manipulation manipulation(nh, planning_group);
  Perception perception(nh);

  perception.transform_listener = TransformListenerPtr(
      new tf::TransformListener());

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

    while (manipulation.getting_grasps)
    {
    }
    
    manipulation.path_planning();

  }

  ros::waitForShutdown();

  return 0;
}