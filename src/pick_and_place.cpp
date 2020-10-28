#include <ros/ros.h>
#include "concatenate_clouds.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pick_and_place");
    ros::NodeHandle nh;

    Concat concat(nh);
    
    ros::spin();
}