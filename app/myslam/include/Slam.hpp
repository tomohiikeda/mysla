#pragma once

#include <pthread.h> 
#include "Sensor.hpp"
#include "Odometer.hpp"
#include "Plotter.hpp"
#include "DirectinalPosition.hpp"
#include "PointCloud.hpp"

class Slam{
    public:
        Slam(Sensor *sensor, Odometer *odometer, Plotter *plotter){
            this->sensor = sensor;
            this->odometer = odometer;
            this->plotter = plotter;
        }
        bool init(void);
        bool start(void);
        void stop(void);

    protected:
        Sensor *sensor;
        Odometer *odometer;
        Plotter *plotter;
        bool running;
        pthread_t slam_thread;
        DirectionalPosition current_position;
        PointCloud world_map;

        static void *process_loop(void *arg);

    private:
};
