#include "perception_class.hpp"

Perception::Perception(ros::NodeHandle nh)
{
    pub = nh.advertise<sensor_msgs::PointCloud2>("/cloud_combined", 1);
    wrist_camera = nh.subscribe("/camera/depth/points", 1, &Perception::callback, this);
}

void Perception::callback(const sensor_msgs::PointCloud2 msg)
{
    pcl::fromROSMsg(msg, this->current_cloud);

    ros::Time stamp = ros::Time(0);
    pcl_conversions::toPCL(stamp, this->current_cloud.header.stamp);
    this->transform_listener->waitForTransform("/world", this->current_cloud.header.frame_id, stamp, ros::Duration(10.0));
    pcl_ros::transformPointCloud("/world", this->current_cloud, this->current_cloud, *this->transform_listener);
}

void Perception::publish()
{
    ROS_WARN("Publishing combined cloud...");
    sensor_msgs::PointCloud2 cloud;
    toROSMsg(this->combined_cloud, cloud);
    pub.publish(cloud);
}

void Perception::concatenate_clouds()
{
    //TODO: add exception when pointcloud is empty...

    PointCloud<PointXYZRGB>::Ptr temp_cloud(new PointCloud<PointXYZRGB>);

    *temp_cloud = this->left_cloud;
    *temp_cloud += this->right_cloud;
    *temp_cloud += this->top_cloud;

    /*
    if (pcl::io::loadPCDFile("/home/kyassini/Desktop/test_cloud.pcd", *temp_cloud) == -1)
    {
        ROS_ERROR("Failed to open");
    }
    ROS_INFO_STREAM("Loaded ");
    */

    passthrough_filter(temp_cloud);
    segmentPlane(temp_cloud);

    this->combined_cloud = *temp_cloud;

    //pcl::io::savePCDFileASCII("test_cloud.pcd", this->combined_cloud);

    publish();
}

void Perception::snapshot_left()
{
    //sub = nh.subscribe("/camera_left/depth/points", 1, &Perception::callback, this);

    this->left_cloud = this->current_cloud;
    ROS_WARN("Left snapshot complete");
}

void Perception::snapshot_right()
{
    // sub = nh.subscribe("/camera_right/depth/points", 1, &Perception::callback, this);

    this->right_cloud = this->current_cloud;
    ROS_WARN("Right snapshot complete");
}

void Perception::snapshot_top()
{
    // sub = nh.subscribe("/camera_top/depth/points", 1, &Perception::callback, this);

    this->top_cloud = this->current_cloud;
    ROS_WARN("Top snapshot complete");
}

void Perception::passthrough_filter(PointCloud<PointXYZRGB>::Ptr cloud)
{
    PassThrough<PointXYZRGB> pass_x;
    pass_x.setInputCloud(cloud);
    pass_x.setFilterFieldName("x");
    pass_x.setFilterLimits(-0.3, 0.3);
    pass_x.filter(*cloud);

    PassThrough<PointXYZRGB> pass_y;
    pass_y.setInputCloud(cloud);
    pass_y.setFilterFieldName("y");
    pass_y.setFilterLimits(-0.15, 0.2);
    pass_y.filter(*cloud);

    PassThrough<PointXYZRGB> pass_z;
    pass_z.setInputCloud(cloud);
    pass_z.setFilterFieldName("z");
    pass_z.setFilterLimits(0.8, 1.0);
    pass_z.filter(*cloud);
}

void Perception::segmentPlane(PointCloud<PointXYZRGB>::Ptr cloud)
{
    // remove table surface
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::SACSegmentation<pcl::PointXYZRGB> seg;

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

    pcl::ExtractIndices<PointXYZRGB> extract_indices;
    extract_indices.setInputCloud(cloud);
    extract_indices.setIndices(inliers);
    extract_indices.setNegative(true);
    extract_indices.filter(*cloud);
}