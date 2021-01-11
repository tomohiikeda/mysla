#pragma once

#include <pthread.h> 
#include <cmath>
#include "ISensor.hpp"
#include "IOdometer.hpp"
#include "IPlotter.hpp"
#include "Pose2D.hpp"
#include "PointCloud.hpp"
#include "ScanMatcher.hpp"

class Slam{
    public:
        Slam(ISensor *sensor, IOdometer *odometer, IPlotter *plotter):
            sensor(sensor),
            odometer(odometer),
            plotter(plotter){
            running = false;
            scan_matcher = new ScanMatcher();
        }
        bool init(void);
        bool start(void);
        void stop(void);
        void process_loop(void);

    private:
        ISensor *sensor;
        IOdometer *odometer;
        IPlotter *plotter;
        ScanMatcher *scan_matcher;
        bool running;
        pthread_t slam_thread;
        Pose2D cur_pose;
        PointCloud world_map;

        void update_world_map(const PointCloud *cur_pc);
        void estimate_cur_pose(const PointCloud *cur_pc);
        double calculate_cost(const PointCloud *cur_pc) const;
        void wait_for_key(void) const;
        Pose2D calc_deviation_from_world(const PointCloud *pc) const;
        void grow_world_map(const PointCloud *cur_pc);
        
        
        inline double to_radian(double degree) const {
            return degree * M_PI / 180;
        }

        static void *thread_entry(void *arg);

};
