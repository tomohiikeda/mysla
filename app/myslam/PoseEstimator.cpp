#include "PoseEstimator.hpp"
#include "RobotConst.hpp"

PoseEstimator::PoseEstimator(double control_period, ScanMatcher& scan_matcher):
    scan_matcher(scan_matcher)
{
    this->current_pose.x = 0.0f;
    this->current_pose.y = 0.0f;
    this->current_pose.direction = 0.0f;

    this->control_period = control_period;
//    this->scan_matcher = scan_matcher;
}

Pose2D PoseEstimator::get_estimated_position(const int16_t od_l, const int16_t od_r, const PointCloud *cur_pc)
{
    //printf("l=%d, r=%d\n", od_l, od_r);
    //estimate_from_odometory(od_l, od_r);
    estimate_from_scan(cur_pc);
    return current_pose;
}

Pose2D PoseEstimator::estimate_from_odometory(const int16_t od_l, const int16_t od_r)
{
    double delta_L_l = (2 * M_PI * WHEEL_RADIUS * od_l) / PULSE_COUNT_PER_CYCLE;
    double delta_L_r = (2 * M_PI * WHEEL_RADIUS * od_r) / PULSE_COUNT_PER_CYCLE;
    double delta_L = (delta_L_r + delta_L_l) / 2;
    double delta_theta = (delta_L_r  - delta_L_l) / WHEEL_BASE;

    current_pose.x += delta_L * cos(current_pose.direction + delta_theta / 2);
    current_pose.y += delta_L * sin(current_pose.direction + delta_theta / 2);
    current_pose.direction += delta_theta;

    return current_pose;
}

Pose2D PoseEstimator::estimate_from_scan(const PointCloud *cur_pc)
{
    Pose2D movement;

    // 初回は前回スキャンが無いので計算を無視
    if (this->pre_pc.size() == 0) {
        goto out;
    }

    this->scan_matcher.set_current_scan(cur_pc);
    this->scan_matcher.set_reference_scan(&this->pre_pc);
    movement = this->scan_matcher.do_scan_matching();
    
    current_pose.move_to(movement);

out:
    cur_pc->copy_to(this->pre_pc);
    return current_pose;
}
