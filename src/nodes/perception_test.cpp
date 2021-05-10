/*****************************************************************
  Unit Test to make sure perception code & filters are working
*****************************************************************/

#include "manipulation_class.hpp"
#include "perception_class.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pick_place_test");

    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(4);
    spinner.start();
    ros::Duration(1.0).sleep();

    std::string planning_group = "arm"; // Gen3 planning group

    Perception perception(nh);

    perception.transform_listener = TransformListenerPtr(
        new tf::TransformListener());

    // Take a snapshot, concatenate, filter, and then publish
    while (ros::ok())
    {
        perception.snapshot_top();
        ros::Duration(1).sleep();
        perception.concatenate_clouds();
        ros::Duration(2).sleep();
    }

    ros::waitForShutdown();

    return 0;
}