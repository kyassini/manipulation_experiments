#ifndef GPD_CLASS_HPP
#define GPD_CLASS_HPP

#include <ros/ros.h>
#include <ros/console.h>
#include "gpd_ros/GraspConfigList.h"

class GPD
{
private:
    ros::Publisher pub;
    ros::Subscriber grasp_config;

public:
    GPD(ros::NodeHandle nodeHandle);
    void callback(const gpd_ros::GraspConfigList msg);
    void test();
    bool planning = true;
};

#endif