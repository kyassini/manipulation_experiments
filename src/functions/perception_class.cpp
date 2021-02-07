/*****************************************************************
  Perception Function Definitions
*****************************************************************/

#include "perception_class.hpp"

/* 
 * Perception constructor, init cloud publisher and camera subscriber 
 */
Perception::Perception(ros::NodeHandle nh)
{
    pub = nh.advertise<sensor_msgs::PointCloud2>("/cloud_combined", 1);
    wrist_camera = nh.subscribe("/camera/depth/points", 1, &Perception::callback, this);
}

/* 
 * Cloud callback, apply transform
 */
void Perception::callback(const sensor_msgs::PointCloud2 msg)
{
    pcl::fromROSMsg(msg, this->current_cloud);

    ros::Time stamp = ros::Time(0);
    tf::StampedTransform transform;

    /* 
     * NOTE: base_link transform for sim and basic workstation testing,
     * world trasnfrom for NERVE workstation 
     */

    pcl_conversions::toPCL(stamp, this->current_cloud.header.stamp);
    try
    {
        this->transform_listener->waitForTransform("base_link", this->current_cloud.header.frame_id, stamp, ros::Duration(10.0));
        this->transform_listener->lookupTransform("base_link", this->current_cloud.header.frame_id, stamp, transform);
    }
    catch (tf::TransformException ex)
    {
        ROS_ERROR("%s", ex.what());
    }
    pcl_ros::transformPointCloud("base_link", this->current_cloud, this->current_cloud, *this->transform_listener);
}

/* 
 * Publish final cloud for grasp detection
 */
void Perception::publish()
{
    ROS_WARN("Publishing combined cloud...");
    sensor_msgs::PointCloud2 cloud;
    toROSMsg(this->combined_cloud, cloud);
    pub.publish(cloud);
}

/* 
 * Combine clouds and filter, after being transfromed
 */
void Perception::concatenate_clouds()
{
    //TODO: add exception when pointcloud is empty...

    PointCloud<PointXYZ>::Ptr temp_cloud(new PointCloud<PointXYZ>);

    // Combine clouds, currently only the top view is used so concatenating is not necessary
    *temp_cloud = this->top_cloud;
    //*temp_cloud += this->right_cloud;
    //*temp_cloud += this->left_cloud;
    //*temp_cloud += this->top_cloud;

    // Code for loading saved pointcloud, for testing purposes
    /*
    if (pcl::io::loadPCDFile("/home/kyassini/Desktop/test_cloud.pcd", *temp_cloud) == -1)
    {
        ROS_ERROR("Failed to open");
    }
    ROS_INFO_STREAM("Loaded ");
    */

    // Apply series of filtering steps to reduce GPD grasp candidate time and isolate object
    passthrough_filter(temp_cloud);
    segmentPlane(temp_cloud);
    voxelGrid(temp_cloud);

    this->combined_cloud = *temp_cloud;

    // Optional: save pointcloud
    //pcl::io::savePCDFileASCII("test_cloud.pcd", this->combined_cloud);

    publish();
}

/*
 * Snapshot functions for each angle, takes image from wrist camera
 */
void Perception::snapshot_left()
{
    this->left_cloud = this->current_cloud;
    ROS_WARN("Left snapshot complete");
}
void Perception::snapshot_right()
{
    this->right_cloud = this->current_cloud;
    ROS_WARN("Right snapshot complete");
}
void Perception::snapshot_top()
{
    this->top_cloud = this->current_cloud;
    ROS_WARN("Top snapshot complete");
}

/*
 * Filtering steps:
 *      Passthrough: isoaltes defined region
 *      segmentPlane: removes table from cloud
 *      voxelGrid: down sample cloud
 */
void Perception::passthrough_filter(PointCloud<PointXYZ>::Ptr cloud)
{
    // Ideal NERVE WORKSTATION PassThrough VALUES:

    /*
    PassThrough<PointXYZ> pass_x;
    pass_x.setInputCloud(cloud);
    pass_x.setFilterFieldName("x");
    pass_x.setFilterLimits(-0.3, 0.3);
    pass_x.filter(*cloud);

    PassThrough<PointXYZ> pass_y;
    pass_y.setInputCloud(cloud);
    pass_y.setFilterFieldName("y");
    pass_y.setFilterLimits(-0.15, 0.2);
    pass_y.filter(*cloud);

    PassThrough<PointXYZ> pass_z;
    pass_z.setInputCloud(cloud);
    pass_z.setFilterFieldName("z");
    pass_z.setFilterLimits(0.8, 1);
    pass_z.filter(*cloud);
    */

    // Kev's home workstation PassThrough values:
    PassThrough<PointXYZ> pass_z;
    pass_z.setInputCloud(cloud);
    pass_z.setFilterFieldName("z");
    pass_z.setFilterLimits(0.00, 0.15); //0, 0.2 //0.03, 0.2
    pass_z.filter(*cloud);
}

void Perception::segmentPlane(PointCloud<PointXYZ>::Ptr cloud)
{
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::SACSegmentation<pcl::PointXYZ> seg;

    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
    seg.setMaxIterations(1000);

    Eigen::Vector3f axis = Eigen::Vector3f(0.0, 0.0, 1.0);
    seg.setAxis(axis);

    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(1000);
    seg.setDistanceThreshold(0.01);
    seg.setEpsAngle(30.0f * (3.14159 / 180.0f)); // 30 degree tolerance

    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefficients);

    pcl::ExtractIndices<PointXYZ> extract_indices;
    extract_indices.setInputCloud(cloud);
    extract_indices.setIndices(inliers);
    extract_indices.setNegative(true);
    extract_indices.filter(*cloud);
}

void Perception::voxelGrid(PointCloud<PointXYZ>::Ptr cloud)
{
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud);
    sor.setLeafSize(0.01f, 0.01f, 0.01f);
    sor.filter(*cloud);
}