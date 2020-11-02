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
  ROS_INFO_STREAM("Planning group is: " << planning_group);

  while (ros::ok())
  {
    // Testing manipulation class...
    manipulation.goTop();
    ros::Duration(3).sleep();
    manipulation.goRight();
    ros::Duration(3).sleep();
    manipulation.goLeft();
    ros::Duration(3).sleep();
    manipulation.goVertical();
    ros::Duration(3).sleep();
  }

  ros::waitForShutdown();

  return 0;
}