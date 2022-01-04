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
    estimate_from_odometory(*slam_data.odometory());
    estimate_from_scan(slam_data.pc());
    return this->current_pose;
}

Pose2D PoseEstimator::estimate_from_odometory(const odometory_t odom)
{
    double delta_L_l = (2 * M_PI * WHEEL_RADIUS * odom.left) / PULSE_COUNT_PER_CYCLE;
    double delta_L_r = (2 * M_PI * WHEEL_RADIUS * odom.right) / PULSE_COUNT_PER_CYCLE;
    double delta_L = (delta_L_r + delta_L_l) / 2;

    double dtheta = (delta_L_r  - delta_L_l) / WHEEL_BASE;
    double dx = -delta_L * sin(current_pose.direction + dtheta / 2);
    double dy = delta_L * cos(current_pose.direction + dtheta / 2);

    current_pose.x += dx;
    current_pose.y += dy;
    current_pose.direction += dtheta;
    //current_pose.print();

    double dxx = -delta_L * sin(dtheta / 2);
    double dyy = delta_L * cos(dtheta / 2);
    this->diff_from_pre.set(dxx, dyy, dtheta);

    return current_pose;
}

Pose2D PoseEstimator::estimate_from_scan(const PointCloud *cur_pc)
{
    Pose2D movement;
    Pose2D move_world;
    double x, y, theta;
    PointCloud pc;

    // 初回は前回スキャンが無いので計算を無視
    if (this->pre_pc.size() == 0)
        goto out;

    cur_pc->copy_to(pc);
    pc.move(this->diff_from_pre);

    movement = scan_matcher.do_scan_matching(&pc, &this->pre_pc, 1.0f);

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
