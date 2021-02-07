/*****************************************************************
  GPD (Grasp Pose Detection) Specific Function Definitions
*****************************************************************/

#include "manipulation_class.hpp"

/* 
 * Get grasp msgs from gpd_ros package 
 */
void Manipulation::callback(const gpd_ros::GraspConfigList msg)
{
    ROS_WARN("Received Grasp Candidates");
    grasp_candidates = msg;
    this->getting_grasps = false;
}

/* 
 * Plan to each grasp candidate in order of score until a valid one is found
 * Assign grasp msg values to class variables
 */
void Manipulation::path_planning()
{
    int n = sizeof(this->grasp_candidates);
    if (n == 0)
    {
        ROS_ERROR("Grasp list is empty");
    }

    this->score = this->grasp_candidates.grasps[0].score;

    int i = 0;
    this->grabbed_object = 0;

    for (i = 0; ((this->score.data > -150) && i < 10); i++)
    {
        this->grasp = this->grasp_candidates.grasps[i];
        this->pose = this->grasp.position;
        this->pose_sample = this->grasp.sample;
        this->orientation = this->grasp.approach;
        this->axis = this->grasp.axis;
        this->score = this->grasp.score;
        ROS_INFO("Grasp score: %f", this->score.data);

        set_target_pose();
        plan_pose_goal();

        if (this->pose_success)
        {
            this->grabbed_object = true;
            ROS_WARN("Plan success");
            break;
        }
    }
}

void Manipulation::set_target_pose()
{
    this->q.setRPY(this->orientation.x - pi, this->orientation.y, this->orientation.z);
    this->q.normalize();
    this->target_pose.orientation = tf2::toMsg(this->q);
    this->target_pose.position.x = this->pose_sample.x;
    this->target_pose.position.y = this->pose_sample.y;
    this->target_pose.position.z = this->pose_sample.z;
}

void Manipulation::plan_pose_goal()
{
    this->move_group_ptr->setPoseTarget(this->target_pose);
    this->move_group_ptr->setGoalPositionTolerance(0.01);
    this->move_group_ptr->setGoalOrientationTolerance(0.002);
    this->move_group_ptr->setPlanningTime(2);
    this->move_group_ptr->setNumPlanningAttempts(30);
    this->pose_success = (this->move_group_ptr->plan(this->my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
}

/* 
 * Working pick_and_place functions:
 *   Removed moveit! pick() pipeline. Instead moving directly to pre and post by
 *   calculating these positions using gpd_grasp_to_pose()
 */
void Manipulation::pickup()
{
    std::vector<geometry_msgs::Pose> poses;

    // Get pre, actual, and post grasps directly from GPD grasp msg
    poses = gpd_grasp_to_pose(grasp);

    //TODO: Add excpetion when any plan fails

    ROS_WARN_STREAM("Moving to pre-grasp...");
    this->target_pose.orientation = poses[0].orientation;
    this->target_pose.position.x = poses[0].position.x;
    this->target_pose.position.y = poses[0].position.y;
    this->target_pose.position.z = poses[0].position.z;
    plan_pose_goal();
    this->move_group_ptr->move();

    ROS_WARN_STREAM("Picking...");
    this->target_pose.orientation = poses[0].orientation;
    this->target_pose.position.x = poses[1].position.x;
    this->target_pose.position.y = poses[1].position.y;
    this->target_pose.position.z = poses[1].position.z;
    plan_pose_goal();
    this->move_group_ptr->move();

    close_gripper();

    ROS_WARN_STREAM("Retreating...");
    this->target_pose.orientation = poses[0].orientation;
    this->target_pose.position.x = poses[0].position.x;
    this->target_pose.position.y = poses[0].position.y;
    this->target_pose.position.z = poses[0].position.z;
    plan_pose_goal();
    this->move_group_ptr->move();
}

void Manipulation::place(float z_dist)
{
    ROS_WARN_STREAM("Placing... ");

    // NOTE: All hard coded values, just places the object to the right of the arm base

    this->q.setRPY(-pi, 0, 0);
    this->target_pose.orientation = tf2::toMsg(this->q);
    this->target_pose.position.x = 0;
    this->target_pose.position.y = -0.28;
    this->target_pose.position.z = z_dist;
    plan_pose_goal();
    this->move_group_ptr->move();

    this->q.setRPY(-pi, 0, 0);
    this->target_pose.orientation = tf2::toMsg(this->q);
    this->target_pose.position.x = 0;
    this->target_pose.position.y = -0.28;
    this->target_pose.position.z = z_dist - 0.15;
    plan_pose_goal();
    this->move_group_ptr->move();

    open_gripper();

    this->q.setRPY(-pi, 0, 0);
    this->target_pose.orientation = tf2::toMsg(this->q);
    this->target_pose.position.x = 0;
    this->target_pose.position.y = -0.28;
    this->target_pose.position.z = z_dist;
    plan_pose_goal();
    this->move_group_ptr->move();
}

/* 
 * Given GPD grasp msg, calculate the pre and post grasps along the appraoch vector
 * Modified from: https://gist.github.com/tkelestemur/60401be131344dae98671b95d46060f8
 */
std::vector<geometry_msgs::Pose> Manipulation::gpd_grasp_to_pose(gpd_ros::GraspConfig &grasp)
{
    geometry_msgs::Pose pose;
    geometry_msgs::Pose pre;
    geometry_msgs::Pose post;
    std::vector<geometry_msgs::Pose> poses;

    tf::StampedTransform tf_base_odom;
    tf::Quaternion q_grasp_base;
    tf::TransformListener listener_;

    tf::Matrix3x3 orientation(-grasp.axis.x, grasp.binormal.x, grasp.approach.x,
                              -grasp.axis.y, grasp.binormal.y, grasp.approach.y,
                              -grasp.axis.z, grasp.binormal.z, grasp.approach.z);

    tf::Vector3 tr_grasp_base(grasp.position.x, grasp.position.y, grasp.position.z);
    tf::Transform tf_grasp_base(orientation, tr_grasp_base);

    // Useless transform?
    try
    {
        listener_.waitForTransform("base_link", "base_link", ros::Time(0), ros::Duration(3.0));
        listener_.lookupTransform("base_link", "base_link", ros::Time(0), tf_base_odom);
    }
    catch (tf::TransformException ex)
    {
        ROS_ERROR("%s", ex.what());
    }

    // Find grasp pose
    tf::Transform tf_grasp_odom = tf_base_odom * tf_grasp_base;
    tf::poseTFToMsg(tf_grasp_odom, pose);

    // Find pre-grasp pose
    tf::Transform tf_pregrasp_odom_(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0, 0, -0.08));
    tf::Transform tf_pregrasp_odom = tf_grasp_odom * tf_pregrasp_odom_;
    tf::poseTFToMsg(tf_pregrasp_odom, pre);

    tf::StampedTransform tf_hand_odom;
    try
    {
        listener_.waitForTransform("base_link", "end_effector_link", ros::Time(0), ros::Duration(3.0));
        listener_.lookupTransform("base_link", "end_effector_link", ros::Time(0), tf_hand_odom);
    }
    catch (tf::TransformException ex)
    {
        ROS_ERROR("%s", ex.what());
    }

    Eigen::VectorXf grasp_position(2);
    Eigen::VectorXf hand_position(2);

    grasp_position << grasp.position.x, grasp.position.y;
    hand_position << tf_hand_odom.getOrigin().getX(), tf_hand_odom.getOrigin().getY();
    float distance = (grasp_position - hand_position).squaredNorm();

    //tf::Quaternion orientation_quat;
    //orientation.getRotation(orientation_quat);
    //tf::quaternionTFToMsg(orientation_quat, pose.orientation);

    //pose.position = grasp.position;

    poses.push_back(pre);
    poses.push_back(pose);
    poses.push_back(post);

    return poses;
}




/****************************************************************************************************
 * ALL CODE UNDERNEATH THIS COMMENT IS UNUSED AND ARE PREVIOUS UNSUCCESSFUL ATTEMPTS AT PICK & PLACE.
 * WILL DELETE WHEN THE ABOVE METHOD IS VALIDATED
 ****************************************************************************************************/

/*
void Manipulation::pickup()
{

    moveit_msgs::Grasp grasp_cand;

    grasp.position.x = grasp.position.x + 0 * grasp.approach.x;
    grasp.position.y = grasp.position.y + 0 * grasp.approach.y;
    grasp.position.z = grasp.position.z + 0; //* grasp.approach.z;

    grasp_cand.id = "grasp";
    grasp_cand.pre_grasp_approach.min_distance = 0.15;
    grasp_cand.pre_grasp_approach.desired_distance = 0.2;

    grasp_cand.post_grasp_retreat.min_distance = 0.01;
    grasp_cand.post_grasp_retreat.desired_distance = 0.15;
    grasp_cand.post_grasp_retreat.direction.header.frame_id = "end_effector_link";
    grasp_cand.post_grasp_retreat.direction.vector.z = -1.0;

    ros::Time grasp_stamp = grasp_candidates.header.stamp;
    grasp_cand.grasp_pose.header.frame_id = "base_link";
    grasp_cand.pre_grasp_approach.direction.header.frame_id = "end_effector_link";

    geometry_msgs::Pose pre;
    geometry_msgs::Pose post;

    std::vector<geometry_msgs::Pose> poses;

    poses = gpd_grasp_to_pose(grasp);

    grasp_cand.grasp_pose.pose = poses[0];

    //this->q.setRPY(-pi, 0, 0);
    //grasp_cand.grasp_pose.pose.orientation = tf2::toMsg(this->q);

    //grasp_cand.grasp_pose.pose.position.x = grasp.position.x;
    //grasp_cand.grasp_pose.pose.position.y = grasp.position.y;
    //grasp_cand.grasp_pose.pose.position.z = grasp.position.z;

    grasp_cand.grasp_quality = grasp.score.data;

    //grasp_cand.pre_grasp_approach.direction.vector.x = grasp.approach.x;
    //grasp_cand.pre_grasp_approach.direction.vector.y = grasp.approach.y;
    grasp_cand.pre_grasp_approach.direction.vector.z = 1; //grasp.approach.z;

    grasp_candidates_.push_front(std::make_pair(grasp_cand, grasp_stamp));

    geometry_msgs::PoseArray grasps_visualization;
    grasps_visualization.header.frame_id = "base_link";

    grasps_visualization.poses.push_back(poses[0]);

    move_group_ptr->setPlanningTime(20.0);
    grasps_visualization_pub_.publish(grasps_visualization);

    //move_group_ptr->planGraspsAndPick() == moveit_msgs::MoveItErrorCodes::SUCCESS;
    openGripper(grasp_cand.pre_grasp_posture);

    closedGripper(grasp_cand.grasp_posture);

    //this->move_group_ptr->pick("object", grasp_cand);

    //move_group_ptr->planGraspsAndPick() == moveit_msgs::MoveItErrorCodes::SUCCESS;

    ROS_ERROR_STREAM("Pre-grasp...");
    this->target_pose.orientation = poses[0].orientation;
    this->target_pose.position.x = poses[0].position.x;
    this->target_pose.position.y = poses[0].position.y;
    this->target_pose.position.z = poses[0].position.z;

    plan_pose_goal();
    this->move_group_ptr->move();

    ROS_ERROR_STREAM("Actual...");
    this->target_pose.orientation = poses[0].orientation;
    this->target_pose.position.x = poses[1].position.x;
    this->target_pose.position.y = poses[1].position.y;
    this->target_pose.position.z = poses[1].position.z;

    plan_pose_goal();
    this->move_group_ptr->move();

    close_gripper();

    ROS_ERROR_STREAM("Post...");
    this->target_pose.orientation = poses[0].orientation;
    this->target_pose.position.x = poses[0].position.x;
    this->target_pose.position.y = poses[0].position.y;
    this->target_pose.position.z = poses[0].position.z;

    plan_pose_goal();
    this->move_group_ptr->move();
}

void Manipulation::place(float z_dist)
{
    ROS_ERROR_STREAM("Placing... ");

    this->q.setRPY(-pi, 0, 0);
    this->target_pose.orientation = tf2::toMsg(this->q);
    this->target_pose.position.x = 0;
    this->target_pose.position.y = -0.28;
    this->target_pose.position.z = z_dist;

    plan_pose_goal();
    this->move_group_ptr->move();

    this->q.setRPY(-pi, 0, 0);
    this->target_pose.orientation = tf2::toMsg(this->q);
    this->target_pose.position.x = 0;
    this->target_pose.position.y = -0.28;
    this->target_pose.position.z = z_dist - 0.15;

    plan_pose_goal();
    this->move_group_ptr->move();

    open_gripper();

    this->q.setRPY(-pi, 0, 0);
    this->target_pose.orientation = tf2::toMsg(this->q);
    this->target_pose.position.x = 0;
    this->target_pose.position.y = -0.28;
    this->target_pose.position.z = z_dist;

    plan_pose_goal();
    this->move_group_ptr->move();
}

std::vector<geometry_msgs::Pose> Manipulation::gpd_grasp_to_pose(gpd_ros::GraspConfig &grasp)
{
    geometry_msgs::Pose pose;
    geometry_msgs::Pose pre;
    geometry_msgs::Pose post;
    std::vector<geometry_msgs::Pose> poses;

    tf::StampedTransform tf_base_odom;
    tf::Quaternion q_grasp_base;
    tf::TransformListener listener_;

    tf::Matrix3x3 orientation(-grasp.axis.x, grasp.binormal.x, grasp.approach.x,
                              -grasp.axis.y, grasp.binormal.y, grasp.approach.y,
                              -grasp.axis.z, grasp.binormal.z, grasp.approach.z);

    tf::Vector3 tr_grasp_base(grasp.position.x, grasp.position.y, grasp.position.z);
    tf::Transform tf_grasp_base(orientation, tr_grasp_base);

    try
    {
        listener_.waitForTransform("base_link", "base_link", ros::Time(0), ros::Duration(3.0));
        listener_.lookupTransform("base_link", "base_link", ros::Time(0), tf_base_odom);
    }
    catch (tf::TransformException ex)
    {
        ROS_ERROR("%s", ex.what());
    }

    // Find grasp pose
    tf::Transform tf_grasp_odom = tf_base_odom * tf_grasp_base;
    tf::poseTFToMsg(tf_grasp_odom, pose);

    // Find pre-grasp pose
    tf::Transform tf_pregrasp_odom_(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0, 0, -0.08));
    tf::Transform tf_pregrasp_odom = tf_grasp_odom * tf_pregrasp_odom_;
    tf::poseTFToMsg(tf_pregrasp_odom, pre);

    tf::StampedTransform tf_hand_odom;
    try
    {
        listener_.waitForTransform("base_link", "end_effector_link", ros::Time(0), ros::Duration(3.0));
        listener_.lookupTransform("base_link", "end_effector_link", ros::Time(0), tf_hand_odom);
    }
    catch (tf::TransformException ex)
    {
        ROS_ERROR("%s", ex.what());
    }

    Eigen::VectorXf grasp_position(2);
    Eigen::VectorXf hand_position(2);

    grasp_position << grasp.position.x, grasp.position.y;
    hand_position << tf_hand_odom.getOrigin().getX(), tf_hand_odom.getOrigin().getY();
    float distance = (grasp_position - hand_position).squaredNorm();

    //tf::Quaternion orientation_quat;
    //orientation.getRotation(orientation_quat);
    //tf::quaternionTFToMsg(orientation_quat, pose.orientation);

    //pose.position = grasp.position;

    poses.push_back(pre);
    poses.push_back(pose);
    poses.push_back(post);
    return poses;
}


void Manipulation::pick_and_place()
{
    std::unique_ptr<actionlib::SimpleActionClient<moveit_msgs::PickupAction>> pick_action_client_;

    ROS_ERROR_STREAM("Picking and placing");
    //ROS_ERROR_STREAM(this->pose_sample);
    ROS_ERROR_STREAM("POSE" << this->pose);
    //ROS_ERROR_STREAM(this->orientation);

    //this->move_group_ptr->setGoalPositionTolerance(0.001);
    //this->move_group_ptr->setGoalOrientationTolerance(0.002);
    //this->move_group_ptr->setMaxVelocityScalingFactor(0.1);
    //this->move_group_ptr->setMaxAccelerationScalingFactor(0.5);

    std::vector<moveit_msgs::Grasp> grasps;
    grasps.resize(1);

    // Setting grasp pose
    grasps[0].grasp_pose.header.frame_id = "base_link"; //base_link
    this->q.setRPY(-pi, 0, 0);
    this->q.normalize();

    ROS_ERROR_STREAM("RPY: ");
    ROS_ERROR_STREAM(this->orientation.x - pi);
    ROS_ERROR_STREAM(this->orientation.y);
    ROS_ERROR_STREAM(this->orientation.z);

    ROS_ERROR_STREAM("AXIS: ");
    ROS_ERROR_STREAM(this->axis);

    tf2::Vector3 test = q.getAxis();
    ROS_ERROR_STREAM("Vector3: ");
    ROS_ERROR_STREAM(test.getX());
    ROS_ERROR_STREAM(test.getY());
    ROS_ERROR_STREAM(test.getZ());

    ROS_ERROR_STREAM("Q: ");
    ROS_ERROR_STREAM(q.getAngle());
    ROS_ERROR_STREAM(q.getX());
    ROS_ERROR_STREAM(q.getY());
    ROS_ERROR_STREAM(q.getZ());
    ROS_ERROR_STREAM(q.getW());

    geometry_msgs::Point adj_pose = this->compute_point(pose, q.getAxis(), 0.12);
    ROS_ERROR_STREAM("NEW POINT: ");
    ROS_ERROR_STREAM(adj_pose);

    grasps[0].grasp_pose.pose.orientation = tf2::toMsg(this->q);
    grasps[0].grasp_pose.pose.position.x = adj_pose.x;
    grasps[0].grasp_pose.pose.position.y = adj_pose.y;
    grasps[0].grasp_pose.pose.position.z = adj_pose.z; //+0.08

    // Setting pre-grasp pose
    grasps[0].pre_grasp_approach.direction.header.frame_id = "end_effector_link";
    grasps[0].pre_grasp_approach.direction.vector.z = 1.0;
    grasps[0].pre_grasp_approach.min_distance = 0.15;    //0.35
    grasps[0].pre_grasp_approach.desired_distance = 0.2; //0.4

    // Setting post-grasp retreat
    grasps[0].post_grasp_retreat.direction.header.frame_id = "end_effector_link";
    grasps[0].post_grasp_retreat.direction.vector.z = -1.0;
    grasps[0].post_grasp_retreat.min_distance = 0.01;     //0.1
    grasps[0].post_grasp_retreat.desired_distance = 0.15; //0.25

    // Setting posture of eef before grasp
    openGripper(grasps[0].pre_grasp_posture);

    closedGripper(grasps[0].grasp_posture);

    this->move_group_ptr->setSupportSurfaceName("table");

    //this->move_group_ptr->move();
    this->move_group_ptr->pick("object", grasps[0]);
}

geometry_msgs::Point Manipulation::compute_point(geometry_msgs::Point obj_point, tf2::Vector3 vector, double offset)
{
    tf2::Vector3 approach;
    geometry_msgs::Point adjusted_point;

    approach.setX(vector.getX() - obj_point.x);
    approach.setY(vector.getY() - obj_point.y);
    approach.setZ(vector.getZ() - obj_point.z);

    adjusted_point.x = obj_point.x; // - offset * vector.getZ();
    adjusted_point.y = obj_point.y; // - offset * vector.getY();
    adjusted_point.z = obj_point.z - offset * vector.getX();

    return adjusted_point;
}
*/