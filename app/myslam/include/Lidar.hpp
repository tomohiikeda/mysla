#pragma once

#include <stdint.h>
#include "rplidar.h"
#include "Plotter.hpp"

using namespace rp::standalone::rplidar;

class Lidar{
    public:
        Lidar(Plotter *plotter):_plotter(plotter){}
        bool init(const char *devname, const uint32_t baudrate);
        void start(void);
        void stop(void);

    protected:
        RPlidarDriver *_drv;
        Plotter *_plotter;

        bool check_health(void);
        bool get_devinfo(void);

    private:
};
