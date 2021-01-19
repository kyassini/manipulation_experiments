#ifndef PERCEPTION_CLASS_HPP
#define PERCEPTION_CLASS_HPP

#include <boost/filesystem.hpp>
#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>	
#include <pcl/filters/passthrough.h>
#include <tf/transform_listener.h>

// planar seg includes
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>


using namespace pcl;

typedef boost::shared_ptr<tf::TransformListener> TransformListenerPtr;

class Perception
{
private:
    ros::Publisher pub;
    ros::Subscriber wrist_camera;


public:
    Perception(ros::NodeHandle nodeHandle);
    TransformListenerPtr transform_listener;

    PointCloud<PointXYZ> combined_cloud;
    PointCloud<PointXYZ> left_cloud, right_cloud, top_cloud, current_cloud;

    void callback(const sensor_msgs::PointCloud2 msg);
    void concatenate_clouds();
    void publish();

    void snapshot_left();
    void snapshot_right();
    void snapshot_top();  

    void passthrough_filter(PointCloud<PointXYZ>::Ptr);  
    void segmentPlane(PointCloud<PointXYZ>::Ptr);  

};

#endif