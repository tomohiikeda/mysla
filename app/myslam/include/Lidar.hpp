#pragma once

#include <stdint.h>
#include "ISensor.hpp"
#include "Lidar.hpp"
#include "rplidar.h"

using namespace rp::standalone::rplidar;

class Lidar : public ISensor{
    public:
        bool init(void);
        bool start(void);
        void stop(void);
        bool get_point_cloud(PointCloud *point_cloud);

    protected:
        RPlidarDriver *_drv;
        bool check_health(void);
        bool get_devinfo(void);

    private:
    
};
