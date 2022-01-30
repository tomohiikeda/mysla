#pragma once

#include "Common.hpp"
#include "PointCloud.hpp"
#include "Pose2D.hpp"
#include "GridMap.hpp"

class IPlotter{
    public:
        enum color {
            red,
            green,
            blue,
            cyan,
            magenta,
            yellow,
            black,
            purple,
        };
        enum point_type {
            NA,
            plus,
            cross,
            star,
            square,
            square_fill,
            round,
            round_fill,
            triangle,
            triangle_fill,
            rev_triangle,
            rev_triangle_fill,
            diamond,
            diamond_fill,
        };
        struct plot_info {
            std::string label;
            enum color color;
            double pt_size;
            enum point_type pt_type;
            double line_width;
        };

        virtual bool open(void) = 0;
        virtual void close(void) = 0;
        virtual void plot(const Pose2D& pose, const PointCloud& pc) const = 0;
        virtual void plot(const Pose2D& pose, const struct IPlotter::plot_info& pose_info, const PointCloud& pc, const struct IPlotter::plot_info& pc_info) const = 0;
        virtual void plot(const PointCloud& pc) const = 0;
        virtual void plot(const PointCloud& pc, const struct IPlotter::plot_info& pc_info) const = 0;
        virtual void plot(const PointCloud& pc_0, const PointCloud& pc_1) const = 0;
        virtual void plot(const PointCloud& pc_0, const struct IPlotter::plot_info& pc_info_0, const PointCloud& pc_1, const struct IPlotter::plot_info& pc_info_1) const = 0;
        virtual void plot(const PointCloud& pc_0, const PointCloud& pc_1, const std::vector<uint32_t>& associate_list) const = 0;
        virtual void plot(const PointCloud& pc_0, const struct IPlotter::plot_info& pc_info_0, const PointCloud& pc_1, const struct IPlotter::plot_info& pc_info_1, const std::vector<uint32_t>& associate_list) const = 0;
        virtual void plot(const Pose2D& pose, const GridMap& grid_map) const = 0;
        virtual void plot(const Pose2D& pose, const struct IPlotter::plot_info& pose_info, const GridMap& grid_map, const struct IPlotter::plot_info& map_info) const = 0;
};

