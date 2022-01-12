#pragma once

#include "Common.hpp"
#include "PointCloud.hpp"

class GridMap{
    public:
        GridMap();
        void set_points(const PointCloud *world_pc);
        void to_point_cloud(PointCloud *to_pc) const;

    protected:
        static const int32_t map_min_x = -3000;
        static const int32_t map_max_x = 3000;
        static const int32_t map_min_y = -3000;
        static const int32_t map_max_y = 3000;
        static const uint32_t grid_per = 5;
        static const uint32_t map_size_x = (map_max_x - map_min_x) / grid_per;
        static const uint32_t map_size_y = (map_max_y - map_min_y) / grid_per;
        uint8_t grid_map[map_size_x][map_size_x];
};
