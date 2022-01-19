#include "PoseEstimator.hpp"
#include "RobotConst.hpp"

PoseEstimator::PoseEstimator(ScanMatcher& scan_matcher):
    scan_matcher(scan_matcher)
{
}

Pose2D PoseEstimator::estimate_position(const Pose2D cur_pose, SlamData& slam_data, const GridMap& world_map) const
{
    Pose2D pose;
    pose = estimate_from_odometory(cur_pose, *slam_data.odometory());
    pose = estimate_from_scan(pose, slam_data.pc(), world_map);
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

    pose.print();

    return pose;
}

Pose2D PoseEstimator::estimate_from_scan(const Pose2D cur_pose, const PointCloud *cur_pc, const GridMap& world_map) const
{
    Pose2D movement;
    Pose2D pose = cur_pose;
    PointCloud pc, ref_scan;
    double min_x = cur_pose.x - 1500;
    double max_x = cur_pose.x + 1500;
    double min_y = cur_pose.y - 1500;
    double max_y = cur_pose.y + 1500;

    world_map.to_point_cloud(&ref_scan);

    // 初回は前回スキャンが無いので計算を無視
    if (ref_scan.size() == 0)
        goto out;

    cur_pc->copy_to(pc);
    //pc.trim(min_x, max_x, min_y, max_y);
    pc.move(pose);

    //ref_scan.trim(min_x, max_x, min_y, max_y);
    ref_scan.analyse_points();
    movement = scan_matcher.do_scan_matching(&pc, &ref_scan, 1.0f);

    pose.move_to(movement);

out:
    pose.print();
    return pose;
}
