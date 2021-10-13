#pragma once

#include "Common.hpp"
#include "Pose2D.hpp"

class PoseEstimator{
    public:
        PoseEstimator(double control_period);
        virtual ~PoseEstimator(void){}
        Pose2D get_estimated_position(int16_t od_l, int16_t od_r);

    protected:
        double control_period;
        Pose2D current_pose;
        Pose2D estimate(int16_t od_l, int16_t od_r);
};
