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

  PointCloud transform(PointCloud cloud)
  {
    listener.waitForTransform("/world", cloud.header.frame_id, ros::Time(0), ros::Duration(10.0));
    pcl_ros::transformPointCloud("/world", cloud, cloud, listener);
    return cloud;
  }

  void pulbish(PointCloud cloud)
  {
    pcl::toROSMsg(cloud, combined_cloud);
    pub.publish(combined_cloud);
  }

public:
  Concat(ros::NodeHandle nh)
  {
    i = 0;
    pub = nh.advertise<PointCloud>("combined_cloud", 1);
    camera_left = nh.subscribe<PointCloud>("camera_left/depth/points", 1, &Concat::callback_left, this);
    camera_right = nh.subscribe<PointCloud>("camera_right/depth/points", 1, &Concat::callback_right, this);
    camera_top = nh.subscribe<PointCloud>("camera_top/depth/points", 1, &Concat::callback_top, this);
    camera_rear = nh.subscribe<PointCloud>("camera_rear/depth/points", 1, &Concat::callback_rear, this);
  }

  void callback_left(const PointCloud::ConstPtr &cloud_msg)
  {
    i = 0;
    add(cloud_msg);
  }
  void callback_right(const PointCloud::ConstPtr &cloud_msg)
  {
    i = 1;
    add(cloud_msg);
  }
  void callback_top(const PointCloud::ConstPtr &cloud_msg)
  {
    i = 2;
    add(cloud_msg);
  }
  void callback_rear(const PointCloud::ConstPtr &cloud_msg)
  {
    i = 3;
    add(cloud_msg);
  }

  void add(const PointCloud::ConstPtr &cloud_msg)
  {
    outputs[i] = *cloud_msg;
    outputs[i] = transform(outputs[i]);
    output_pcl = outputs[0];
    for (int n = 1; n < sizeof(outputs) / sizeof(PointCloud); n++)
    {
      output_pcl += outputs[n];
    }
    pulbish(output_pcl);
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "concatenate_clouds");
  ros::NodeHandle nh;

  Concat concat(nh);

  ros::spin();
}