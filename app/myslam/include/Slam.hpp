#pragma once

#include <pthread.h> 
#include "ISensor.hpp"
#include "IOdometer.hpp"
#include "IPlotter.hpp"
#include "DirectinalPosition.hpp"
#include "PointCloud.hpp"

class Slam{
    public:
        Slam(ISensor *sensor, IOdometer *odometer, IPlotter *plotter){
            this->sensor = sensor;
            this->odometer = odometer;
            this->plotter = plotter;
            running = false;
        }
        bool init(void);
        bool start(void);
        void stop(void);
        void process_loop(void);

    protected:
        ISensor *sensor;
        IOdometer *odometer;
        IPlotter *plotter;
        bool running;
        pthread_t slam_thread;
        DirectionalPosition cur_pos;
        PointCloud world_map;

        void update_world_map(PointCloud& cur_pc);
        void estimate_cur_pos(PointCloud& cur_pc);

        static void *thread_entry(void *arg);

    private:
};
