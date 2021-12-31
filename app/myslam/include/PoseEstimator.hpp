#pragma once

#include "Common.hpp"
#include "Pose2D.hpp"
#include "ScanMatcher.hpp"
#include "GlidMap.hpp"

struct odometory {
    int16_t left;
    int16_t right;
};

struct estimate_data {
    struct odometory odom;
    PointCloud *pc;
};

class PoseEstimator{
    public:
        PoseEstimator(ScanMatcher& scan_matcher);
        virtual ~PoseEstimator(void){}
        Pose2D estimate_position(const struct estimate_data *est_data);
        Pose2D estimate_position(const PointCloud *pc);

    protected:
        ScanMatcher& scan_matcher;
        Pose2D current_pose;
        Pose2D estimate_from_odometory(const int16_t od_l, const int16_t od_r);
        Pose2D estimate_from_scan(const PointCloud *cur_pc);
        PointCloud pre_pc;
};
