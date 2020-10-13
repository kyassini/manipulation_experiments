#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <tf/transform_listener.h>

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;

// Merged Pointcloud class, TODO: use class to return transformed cloud to add to other cloud objects later
class Concat
{
private:
  ros::NodeHandle nh;

  ros::Publisher pub = nh.advertise<PointCloud>("output", 1);

  ros::Subscriber camera_left = nh.subscribe<PointCloud>("camera_left/depth/points", 1, &Concat::callback1, this);
  ros::Subscriber camera_right = nh.subscribe<PointCloud>("camera_right/depth/points", 1, &Concat::callback2, this);
  ros::Subscriber camera_top = nh.subscribe<PointCloud>("camera_top/depth/points", 1, &Concat::callback3, this);
  ros::Subscriber camera_rear = nh.subscribe<PointCloud>("camera_rear/depth/points", 1, &Concat::callback4, this);

  PointCloud output_pcl, output1_pcl, output2_pcl, output3_pcl, output4_pcl;
  sensor_msgs::PointCloud2 output;
  tf::TransformListener listener;

  PointCloud listen(PointCloud cloud)
  {
    listener.waitForTransform("/world", cloud.header.frame_id, ros::Time(0), ros::Duration(10.0));
    pcl_ros::transformPointCloud("/world", cloud, cloud, listener);
    return cloud;
  }

  void pulbish(PointCloud cloud)
  {
    pcl::toROSMsg(cloud, output);
    pub.publish(output);
  }

public:
  void callback1(const PointCloud::ConstPtr &cloud_msg)
  {
    output1_pcl = *cloud_msg;
    output1_pcl = listen(output1_pcl);
    add();
  }
  void callback2(const PointCloud::ConstPtr &cloud_msg)
  {
    output2_pcl = *cloud_msg;
    output2_pcl = listen(output2_pcl);
    add();
  }
  void callback3(const PointCloud::ConstPtr &cloud_msg)
  {
    output3_pcl = *cloud_msg;
    output3_pcl = listen(output3_pcl);
    add();
  }
  void callback4(const PointCloud::ConstPtr &cloud_msg)
  {
    output4_pcl = *cloud_msg;
    output4_pcl = listen(output4_pcl);
    add();
  }

  void add()
  {
    output_pcl = output1_pcl;
    output_pcl += output2_pcl;
    output_pcl += output3_pcl;
    output_pcl += output4_pcl;
    pulbish(output_pcl);
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "concatenate_clouds");

  Concat concat;

  ros::spin();
}