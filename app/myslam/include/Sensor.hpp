#pragma once

#include "PointCloud.hpp"

class Sensor{
    public:
        virtual bool init(void) = 0;
        virtual bool start(void) = 0;
        virtual void stop(void) = 0;
        virtual bool get_point_cloud(PointCloud& point_cloud) = 0;

    protected:

    private:
};
