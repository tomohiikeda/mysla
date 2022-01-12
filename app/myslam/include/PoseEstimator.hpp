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
        Pose2D estimate_position(SlamData& slam_data, const GridMap& world_map);

    protected:
        ScanMatcher& scan_matcher;
        Pose2D current_pose;
        Pose2D estimate_from_odometory(const odometory_t odom);
        Pose2D estimate_from_scan(const PointCloud *cur_pc, const GridMap& world_map);
        PointCloud pre_pc;
        Pose2D diff_from_pre;
};
