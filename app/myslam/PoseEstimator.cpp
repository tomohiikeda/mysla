#include "PoseEstimator.hpp"
#include "RobotConst.hpp"

PoseEstimator::PoseEstimator(ScanMatcher& scan_matcher):
    scan_matcher(scan_matcher)
{
    this->current_pose.x = 0.0f;
    this->current_pose.y = 0.0f;
    this->current_pose.direction = to_radian(0);
}

Pose2D PoseEstimator::estimate_position(SlamData& slam_data, const GridMap& world_map)
{
    estimate_from_odometory(*slam_data.odometory());
    estimate_from_scan(slam_data.pc(), world_map);
    return this->current_pose;
}

Pose2D PoseEstimator::estimate_from_odometory(const odometory_t odom)
{
    double delta_L_l = (2 * M_PI * WHEEL_RADIUS * odom.left) / PULSE_COUNT_PER_CYCLE;
    double delta_L_r = (2 * M_PI * WHEEL_RADIUS * odom.right) / PULSE_COUNT_PER_CYCLE;
    double delta_L = (delta_L_r + delta_L_l) / 2;

    // 世界座標系の移動量
    double dtheta = (delta_L_r  - delta_L_l) / WHEEL_BASE;
    double dx = -delta_L * sin(current_pose.direction + dtheta / 2);
    double dy = delta_L * cos(current_pose.direction + dtheta / 2);

    current_pose.x += dx;
    current_pose.y += dy;
    current_pose.direction += dtheta;

    return current_pose;
}

Pose2D PoseEstimator::estimate_from_scan(const PointCloud *cur_pc, const GridMap& world_map)
{
    Pose2D movement;
    PointCloud pc, ref_scan;

    // 初回は前回スキャンが無いので計算を無視
    if (this->pre_pc.size() == 0)
        goto out;

    cur_pc->copy_to(pc);
    pc.move(current_pose);

    // 世界地図から参照スキャンを抽出
    world_map.to_point_cloud(&ref_scan);
    ref_scan.analyse_points();
    movement = scan_matcher.do_scan_matching(&pc, &ref_scan, 1.0f);

    current_pose.move_to(movement);

out:
    cur_pc->copy_to(this->pre_pc);
    return current_pose;
}
