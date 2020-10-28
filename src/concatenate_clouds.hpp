// Concatenate_clouds.hpp
#ifndef CONCATENATE_CLOUDS_HPP
#define CONCATENATE_CLOUDS_HPP

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <tf/transform_listener.h>

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;

// Merged Pointcloud class, TODO: reduce callbacks into one, reorganize class...
class Concat
{
private:
  ros::Publisher pub;
  ros::Subscriber camera_left, camera_right, camera_top, camera_rear;

  PointCloud output_pcl;
  PointCloud outputs[4];

  sensor_msgs::PointCloud2 combined_cloud;
  tf::TransformListener listener;

  int i;

  PointCloud transform(PointCloud cloud);

  void pulbish(PointCloud cloud);

public:
  Concat(ros::NodeHandle nh);
  
  void callback_left(const PointCloud::ConstPtr &cloud_msg);
  void callback_right(const PointCloud::ConstPtr &cloud_msg);
  void callback_top(const PointCloud::ConstPtr &cloud_msg);
  void callback_rear(const PointCloud::ConstPtr &cloud_msg);

  void add(const PointCloud::ConstPtr &cloud_msg);
};

#endif