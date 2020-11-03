#include "perception_class.hpp"

Perception::Perception(ros::NodeHandle nh)
{
    pub = nh.advertise<sensor_msgs::PointCloud2>("combined_cloud", 1);
    wrist_camera = nh.subscribe("/camera/depth/points", 1, &Perception::callback, this);
}

void Perception::callback(const sensor_msgs::PointCloud2 msg)
{
    fromROSMsg(msg, this->current_cloud);

    ros::Time stamp = ros::Time(0);
    pcl_conversions::toPCL(stamp, this->current_cloud.header.stamp);
    transform_listener.waitForTransform("/world", current_cloud.header.frame_id, ros::Time(0), ros::Duration(10.0));
    pcl_ros::transformPointCloud("/world", current_cloud, current_cloud, transform_listener);
}

void Perception::publish()
{
    pub.publish(combined_cloud);
}

void Perception::concatenate_clouds()
{
    PointCloud::Ptr temp_cloud(new PointCloud);

    *temp_cloud = this->left_cloud;
    *temp_cloud += this->right_cloud;
    *temp_cloud += this->top_cloud;

    combined_cloud = *temp_cloud;

    publish();
}

void Perception::snapshot_left()
{
    left_cloud = current_cloud;
}

void Perception::snapshot_right()
{
    right_cloud = current_cloud;
}

void Perception::snapshot_top()
{
    top_cloud = top_cloud;
}
