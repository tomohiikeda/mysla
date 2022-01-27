#pragma once

#include "Common.hpp"
#include "Pose2D.hpp"
#include "ScanMatcher.hpp"
#include "GridMap.hpp"
#include "SlamData.hpp"

class PoseEstimator{
    public:
        PoseEstimator(ScanMatcher& scan_matcher);
        virtual ~PoseEstimator(void){}
        Pose2D estimate_position(const Pose2D cur_pose, SlamData& slam_data, const GridMap& world_map) const;

    protected:
        ScanMatcher& scan_matcher;
        Pose2D estimate_from_odometory(const Pose2D cur_pose, const odometory_t odom) const;
        Pose2D estimate_from_scan(const Pose2D cur_pose, PointCloud *cur_pc, const GridMap& world_map) const;
        PointCloud pre_pc;
        Pose2D diff_from_pre;
};
