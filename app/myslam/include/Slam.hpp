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
        }
        bool init(void);
        bool start(void);
        void stop(void);

    protected:
        ISensor *sensor;
        IOdometer *odometer;
        IPlotter *plotter;
        bool running;
        pthread_t slam_thread;
        DirectionalPosition current_position;
        PointCloud world_map;

        static void *process_loop(void *arg);

    private:
};
