#include "PoseEstimator.hpp"
#include "RobotConst.hpp"

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
    //printf("l=%d, r=%d\n", od_l, od_r);
    double delta_L_l = (2 * M_PI * WHEEL_RADIUS * od_l) / PULSE_COUNT_PER_CYCLE;
    double delta_L_r = (2 * M_PI * WHEEL_RADIUS * od_r) / PULSE_COUNT_PER_CYCLE;
    double delta_L = (delta_L_r + delta_L_l) / 2;
    double delta_theta = (delta_L_r  - delta_L_l) / WHEEL_BASE;

    current_pose.x += delta_L * cos(current_pose.direction + delta_theta / 2);
    current_pose.y += delta_L * sin(current_pose.direction + delta_theta / 2);
    current_pose.direction += delta_theta;

    return current_pose;
}