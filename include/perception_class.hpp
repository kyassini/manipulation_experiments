#ifndef PERCEPTION_CLASS_HPP
#define PERCEPTION_CLASS_HPP

#include <boost/filesystem.hpp>
#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <tf/transform_listener.h>

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;

class Perception
{
private:
    ros::Publisher pub;
    ros::Subscriber wrist_camera;
    PointCloud combined_cloud;
    PointCloud left_cloud, right_cloud, top_cloud, current_cloud;

    tf::TransformListener transform_listener;

public:
    Perception(ros::NodeHandle nodeHandle);

    void callback(const sensor_msgs::PointCloud2 msg);
    void concatenate_clouds();
    void publish();

    void snapshot_left();
    void snapshot_right();
    void snapshot_top();
};

#endif