#include "PoseEstimator.hpp"
#include "RobotConst.hpp"

PoseEstimator::PoseEstimator(ScanMatcher& scan_matcher):
    scan_matcher(scan_matcher)
{
    this->current_pose.x = 0.0f;
    this->current_pose.y = 0.0f;
    this->current_pose.direction = 0.0f;
}

Pose2D PoseEstimator::estimate_position(SlamData& slam_data)
{
    //estimate_from_odometory(od_l, od_r);
    estimate_from_scan(slam_data.pc());
    return this->current_pose;
}

Pose2D PoseEstimator::estimate_from_odometory(const int16_t od_l, const int16_t od_r)
{
    double delta_L_l = (2 * M_PI * WHEEL_RADIUS * od_l) / PULSE_COUNT_PER_CYCLE;
    double delta_L_r = (2 * M_PI * WHEEL_RADIUS * od_r) / PULSE_COUNT_PER_CYCLE;
    double delta_L = (delta_L_r + delta_L_l) / 2;
    double delta_theta = (delta_L_r  - delta_L_l) / WHEEL_BASE;

    current_pose.x -= delta_L * sin(current_pose.direction + delta_theta / 2);
    current_pose.y += delta_L * cos(current_pose.direction + delta_theta / 2);
    current_pose.direction += delta_theta;

    return current_pose;
}

Pose2D PoseEstimator::estimate_from_scan(const PointCloud *cur_pc)
{
    Pose2D movement;
    Pose2D move_world;
    double x, y, theta;

    // 初回は前回スキャンが無いので計算を無視
    if (this->pre_pc.size() == 0)
        goto out;

    movement = scan_matcher.do_scan_matching(cur_pc, &this->pre_pc, 1.0f);

    theta = current_pose.direction + movement.direction;
    x = movement.x * cos(theta) - movement.y * sin(theta);
    y = movement.x * sin(theta) + movement.y * cos(theta);
    move_world.x = x;
    move_world.y = y;
    move_world.direction = movement.direction;
    
    current_pose.move_to(move_world);

out:
    cur_pc->copy_to(this->pre_pc);
    return current_pose;
}
