#include <ros/ros.h>
#include "concatenate_clouds.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pick_and_place");
    ros::NodeHandle nh;
    ros::Rate loop_rate(1);
    Concat concat(nh);

    while (ros::ok())
    {
        concat.pulbish();
        ros::spinOnce();
        loop_rate.sleep();
    }

    ros::waitForShutdown();

    return 0;
}