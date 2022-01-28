#include "PoseEstimator.hpp"
#include "RobotConst.hpp"

PoseEstimator::PoseEstimator(ScanMatcher& scan_matcher):
    scan_matcher(scan_matcher)
{
}

Pose2D PoseEstimator::estimate_position(const Pose2D cur_pose, SlamData& slam_data, const GridMap& world_map) const
{
    Pose2D pose = cur_pose;
    pose = estimate_from_odometory(pose, *slam_data.odometory());
    pose = estimate_from_scan(pose, *slam_data.pc(), world_map);
    return pose;
}

Pose2D PoseEstimator::estimate_from_odometory(const Pose2D cur_pose, const odometory_t odom) const
{
    Pose2D pose = cur_pose;
    double delta_L_l = (2 * M_PI * WHEEL_RADIUS * odom.left) / PULSE_COUNT_PER_CYCLE;
    double delta_L_r = (2 * M_PI * WHEEL_RADIUS * odom.right) / PULSE_COUNT_PER_CYCLE;
    double delta_L = (delta_L_r + delta_L_l) / 2;

    // 世界座標系の移動量
    double dtheta = (delta_L_r  - delta_L_l) / WHEEL_BASE;
    double dx = -delta_L * sin(cur_pose.direction + dtheta / 2);
    double dy = delta_L * cos(cur_pose.direction + dtheta / 2);

    pose.x += dx;
    pose.y += dy;
    pose.direction += dtheta;

    return pose;
}

Pose2D PoseEstimator::estimate_from_scan(const Pose2D cur_pose, PointCloud& cur_pc, const GridMap& world_map) const
{
    Movement2D movement;
    Pose2D pose = cur_pose;
    PointCloud pc, ref_scan;
    double offset = 1500.0f;
    double min_x = cur_pose.x - offset;
    double max_x = cur_pose.x + offset;
    double min_y = cur_pose.y - offset;
    double max_y = cur_pose.y + offset;

    world_map.to_point_cloud(&ref_scan, min_x, max_x, min_y, max_y);
    //ref_scan.analyse_points();

    // 初回は前回スキャンが無いので計算を無視
    if (ref_scan.size() == 0)
        goto out;

    cur_pc.copy_to(pc);
    pc.trim(-offset, offset, -offset, offset);
    pc.move(pose);

    movement = scan_matcher.do_scan_matching(&pc, &ref_scan, 1.0f);

    cur_pc.move(pose);

    pose.move(movement);
    cur_pc.move(movement);

out:
    return pose;
}
