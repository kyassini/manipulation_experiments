#include "gpd_class.hpp"

GPD::GPD(ros::NodeHandle nh)
{
    //pub = nh.advertise<gpd_ros::GraspConfigList>("/detect_grasps/clustered_grasps", 1);
    grasp_config = nh.subscribe("/detect_grasps/clustered_grasps", 1, &GPD::callback, this);
}

void GPD::callback(const gpd_ros::GraspConfigList msg)
{
    ROS_ERROR_STREAM("Msg: " << msg);
}

void GPD::test()
{
    ROS_ERROR("Test");
}