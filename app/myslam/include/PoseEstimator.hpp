#pragma once

#include "Common.hpp"
#include "Pose2D.hpp"
#include "ScanMatcher.hpp"
#include "GlidMap.hpp"
#include "SlamData.hpp"

class PoseEstimator{
    public:
        PoseEstimator(ScanMatcher& scan_matcher);
        virtual ~PoseEstimator(void){}
        Pose2D estimate_position(SlamData& slam_data);

    protected:
        ScanMatcher& scan_matcher;
        Pose2D current_pose;
        Pose2D estimate_from_odometory(const int16_t od_l, const int16_t od_r);
        Pose2D estimate_from_scan(const PointCloud *cur_pc);
        PointCloud pre_pc;
};
