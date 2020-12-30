#include "manipulation_class.hpp"
#include "perception_class.hpp"
#include "gpd_class.hpp"

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

  GPD gpd(nh);

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

    while (gpd.planning)
    {
      //gpd.test();
    }
  }

  ros::waitForShutdown();

  return 0;
}