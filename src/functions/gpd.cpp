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
    this->move_group_ptr->setGoalPositionTolerance(0.001);
    this->move_group_ptr->setGoalOrientationTolerance(0.002);
    this->move_group_ptr->setPlanningTime(20);
    this->move_group_ptr->setNumPlanningAttempts(30);
    this->pose_success = (this->move_group_ptr->plan(this->my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
}
