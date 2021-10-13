#include "PoseEstimator.hpp"

PoseEstimator::PoseEstimator(double control_period)
{
    this->current_pose.x = 0.0f;
    this->current_pose.y = 0.0f;
    this->current_pose.direction = 0.0f;
    this->control_period = control_period;
}

Pose2D PoseEstimator::get_estimated_position(int16_t od_l, int16_t od_r)
{
    return estimate(od_l, od_r);
}

Pose2D PoseEstimator::estimate(int16_t od_l, int16_t od_r)
{
    printf("l=%d, r=%d\n", od_l, od_r);
    Pose2D cur_pose(0,0,0);
    return cur_pose;
}