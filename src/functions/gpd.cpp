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

    // Plan to the top grasps to see if they are possible
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
    /* Log algorithim time and grasp score, for testing */
    //std::ofstream logfile;
    //logfile.open("/home/kyassini/Desktop/Testing_Logs/test.txt", std::ios_base::app);

    std::vector<geometry_msgs::Pose> poses;

    // Get pre, actual, and post grasps directly from GPD grasp msg
    poses = gpd_grasp_to_pose(grasp);

    //TODO: Add exception when a plan fails

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

    this->end = ros::Time::now();    // Get times, for testing

    close_gripper();

    ROS_WARN_STREAM("Retreating...");
    this->target_pose.orientation = poses[0].orientation;
    this->target_pose.position.x = poses[0].position.x;
    this->target_pose.position.y = poses[0].position.y;
    this->target_pose.position.z = poses[0].position.z;
    plan_pose_goal();
    this->move_group_ptr->move();


    //logfile << std::endl << this->end - this-> begin << " , " << grasp.score.data; // For testing
    //logfile.close();
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
/*
    this->q.setRPY(-pi, 0, 0);
    this->target_pose.orientation = tf2::toMsg(this->q);
    this->target_pose.position.x = 0;
    this->target_pose.position.y = -0.28;
    this->target_pose.position.z = z_dist - 0.15;
    plan_pose_goal();
    this->move_group_ptr->move();
*/
    open_gripper();

/*
    this->q.setRPY(-pi, 0, 0);
    this->target_pose.orientation = tf2::toMsg(this->q);
    this->target_pose.position.x = 0;
    this->target_pose.position.y = -0.28;
    this->target_pose.position.z = z_dist;
    plan_pose_goal();
    this->move_group_ptr->move();
    */
}

/* 
 * Given GPD grasp msg, calculate the pre and post grasps along the approach vector
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
    //                                      CHANGE THIS VALUE TO EDIT PREGRASP HEIGHT |
    //                                                                                V              
    tf::Transform tf_pregrasp_odom_(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0, 0, -0.13));
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