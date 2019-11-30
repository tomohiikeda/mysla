#pragma once

#include <pthread.h> 
#include <cmath>
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

    private:
        ISensor *sensor;
        IOdometer *odometer;
        IPlotter *plotter;
        bool running;
        pthread_t slam_thread;
        DirectionalPosition cur_pos;
        PointCloud world_map;

        void update_world_map(const PointCloud& cur_pc);
        void estimate_cur_pos(const PointCloud& cur_pc);
        double calculate_cost(const PointCloud& cur_pc) const;
        void wait_for_key(void) const;
        inline double to_radian(double degree) const {
            return degree * M_PI / 180;
        }

        static void *thread_entry(void *arg);

};
