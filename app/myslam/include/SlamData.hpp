#pragma once

#include "Common.hpp"
#include "PointCloud.hpp"
#include "IOdometer.hpp"

class SlamData {

    public:
        SlamData(void);
        double timestamp;
        PointCloud *pc(void);
        odometory_t *odometory(void);
        void save_to_file(const char *filename) const;
        bool load_from_file(const char *filename);
        void print(void) const;

    protected:
        PointCloud point_cloud;
        odometory_t odom;
};
