#include "concatenate_clouds.hpp"

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;

// Merged Pointcloud class, TODO: reduce callbacks into one, reorganize class...

PointCloud Concat::transform(PointCloud cloud)
{
  listener.waitForTransform("/world", cloud.header.frame_id, ros::Time(0), ros::Duration(10.0));
  pcl_ros::transformPointCloud("/world", cloud, cloud, listener);
  return cloud;
}

void Concat::pulbish()
{
  pcl::toROSMsg(output_pcl, combined_cloud);
  pub.publish(combined_cloud);
}

Concat::Concat(ros::NodeHandle nh)
{
  i = 0;
  pub = nh.advertise<PointCloud>("combined_cloud", 1);
  camera_left = nh.subscribe<PointCloud>("camera_left/depth/points", 1, &Concat::callback_left, this);
  camera_right = nh.subscribe<PointCloud>("camera_right/depth/points", 1, &Concat::callback_right, this);
  camera_top = nh.subscribe<PointCloud>("camera_top/depth/points", 1, &Concat::callback_top, this);
  camera_rear = nh.subscribe<PointCloud>("camera_rear/depth/points", 1, &Concat::callback_rear, this);
}

void Concat::callback_left(const PointCloud::ConstPtr &cloud_msg)
{
  i = 0;
  add(cloud_msg);
}
void Concat::callback_right(const PointCloud::ConstPtr &cloud_msg)
{
  i = 1;
  add(cloud_msg);
}
void Concat::callback_top(const PointCloud::ConstPtr &cloud_msg)
{
  i = 2;
  add(cloud_msg);
}
void Concat::callback_rear(const PointCloud::ConstPtr &cloud_msg)
{
  i = 3;
  add(cloud_msg);
}

void Concat::add(const PointCloud::ConstPtr &cloud_msg)
{
  outputs[i] = *cloud_msg;
  outputs[i] = transform(outputs[i]);
  output_pcl = outputs[0];
  for (int n = 1; n < sizeof(outputs) / sizeof(PointCloud); n++)
  {
    output_pcl += outputs[n];
  }
}
