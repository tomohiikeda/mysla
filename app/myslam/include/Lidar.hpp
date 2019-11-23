#pragma once

#include <stdint.h>
#include "rplidar.h"
#include "Plotter.hpp"

using namespace rp::standalone::rplidar;

class Lidar{
    public:
        Lidar(Plotter *plotter){
            _plotter = plotter;
            scan_running = false;
        }
        bool init(const char *devname, const uint32_t baudrate);
        bool start(void);
        void stop(void);

    protected:
        RPlidarDriver *_drv;
        Plotter *_plotter;
        bool scan_running;

        bool check_health(void);
        bool get_devinfo(void);
        pthread_t _scan_thread;
        void print_nodes(rplidar_response_measurement_node_hq_t *nodes, 
                         size_t count);
        void plot_nodes(rplidar_response_measurement_node_hq_t *nodes, 
                        size_t count);

        static void *scan_loop(void *arg);
        
    private:
};
