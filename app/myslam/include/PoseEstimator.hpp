#pragma once

#include "Common.hpp"
#include "Pose2D.hpp"
#include "ScanMatcher.hpp"

class PoseEstimator{
    public:
        PoseEstimator(double control_period, ScanMatcher& scan_matcher);
        virtual ~PoseEstimator(void){}
        Pose2D get_estimated_position(const int16_t od_l, const int16_t od_r, const PointCloud *cur_pc);

    protected:
        double control_period;
        ScanMatcher& scan_matcher;
        Pose2D current_pose;
        Pose2D estimate_from_odometory(const int16_t od_l, const int16_t od_r);
        Pose2D estimate_from_scan(const PointCloud *cur_pc);
        PointCloud pre_pc;
};
