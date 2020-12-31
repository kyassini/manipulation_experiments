#include "manipulation_class.hpp"

void Manipulation::callback(const gpd_ros::GraspConfigList msg)
{
    ROS_WARN("Received Grasp Candidates");
    grasp_candidates = msg;
    this->getting_grasps = false;
}

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
        this->score = this->grasp.score;
        ROS_INFO("Grasp score: %f", this->score.data);

        set_target_pose();
        plan_pose_goal();

        if (this->pose_success)
        {
            this->grabbed_object = true;
            ROS_INFO("Plan success");
            break;
        }
    }
}

void Manipulation::set_target_pose()
{
    this->q.setRPY(this->orientation.x - 3.14, this->orientation.y, this->orientation.z);
    this->q.normalize();
    this->target_pose.orientation = tf2::toMsg(this->q);
    this->target_pose.position.x = this->pose_sample.x;
    this->target_pose.position.y = this->pose_sample.y;
    this->target_pose.position.z = this->pose_sample.z;
}

void Manipulation::plan_pose_goal()
{
    this->move_group_ptr->setPoseTarget(target_pose);
    this->move_group_ptr->setGoalPositionTolerance(0.01);
    this->move_group_ptr->setGoalOrientationTolerance(0.02);
    this->move_group_ptr->setPlanningTime(5);
    this->move_group_ptr->setNumPlanningAttempts(30);
    this->pose_success = (this->move_group_ptr->plan(this->my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
}

void Manipulation::pick_and_place()
{
    ROS_ERROR_STREAM("Picking and placing");
    
    this->move_group_ptr->setGoalPositionTolerance(0.001);
    this->move_group_ptr->setGoalOrientationTolerance(0.002);

    std::vector<moveit_msgs::Grasp> grasps;
    grasps.resize(1);

    grasps[0].grasp_pose.header.frame_id = "base_link";
    this->q.setRPY(this->orientation.x - 3.14, this->orientation.y, this->orientation.z);
    this->q.normalize();
    grasps[0].grasp_pose.pose.orientation = tf2::toMsg(this->q);
    grasps[0].grasp_pose.pose.position.x = this->pose_sample.x;
    grasps[0].grasp_pose.pose.position.y = this->pose_sample.y;
    grasps[0].grasp_pose.pose.position.z = this->pose_sample.z;

    grasps[0].pre_grasp_approach.direction.header.frame_id = "base_link";
    /* Direction is set as positive x axis */
    grasps[0].pre_grasp_approach.direction.vector.z = 1.0;
    grasps[0].pre_grasp_approach.min_distance = 0;
    grasps[0].pre_grasp_approach.desired_distance = 0.1;

    grasps[0].post_grasp_retreat.direction.header.frame_id = "base_link";
    this->move_group_ptr->move();
}