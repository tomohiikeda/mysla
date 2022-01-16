#pragma once

#include "Common.hpp"
#include "PointCloud.hpp"

class Grid {
    public:
        static constexpr uint32_t width = 30;
        static constexpr uint32_t height = 30;
        Grid(void) {};
        void set(const Point p);
        Point get(void) const;
        bool is_valid(void) const;

    protected:
        Point represent;
        uint32_t point_num = 0;
};

class GridMap {
    public:
        GridMap(void) {};
        void set_points(const PointCloud *world_pc);
        void to_point_cloud(PointCloud *to_pc) const;

    protected:
        static constexpr int32_t map_min_x = -3000;
        static constexpr int32_t map_max_x = 3000;
        static constexpr int32_t map_min_y = -3000;
        static constexpr int32_t map_max_y = 3000;
        static constexpr uint32_t max_grid_index_x = (map_max_x - map_min_x) / Grid::width;
        static constexpr uint32_t max_grid_index_y = (map_max_y - map_min_y) / Grid::height;
        int32_t to_index_x(const double world_x) const;
        int32_t to_index_y(const double world_y) const;
        Grid grid_map[max_grid_index_x][max_grid_index_y];
};
