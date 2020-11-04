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

  while (ros::ok())
  {
    // Testing manipulation class...
    manipulation.goTop();
    ros::Duration(3).sleep();
    perception.snapshot_top();
  
    manipulation.goRight();
    ros::Duration(3).sleep();
    perception.snapshot_right();

    manipulation.goLeft();
    ros::Duration(3).sleep();
    perception.snapshot_left();

    perception.concatenate_clouds();
    ros::Duration(3).sleep();
    manipulation.goVertical();

    break;
  }

  ros::waitForShutdown();

  return 0;
}