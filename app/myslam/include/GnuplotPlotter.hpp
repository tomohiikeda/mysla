#pragma once

#include "IPlotter.hpp"
#include "PointCloud.hpp"

class GnuplotPlotter: public IPlotter{
    public:
        bool open(void);
        void close(void);
        void plot(const Pose2D& pose, const PointCloud& pc) const;
        void plot(const Pose2D& pose, const struct IPlotter::plot_info& pose_info,
                  const PointCloud& pc, const struct IPlotter::plot_info& pc_info) const;
        void plot(const Pose2D& pose, const PointCloud& pc_0, const PointCloud& pc_1) const;
        void plot(const Pose2D& pose, const struct IPlotter::plot_info& pose_info,
                  const PointCloud& pc_0, const struct IPlotter::plot_info& pc_info_0,
                  const PointCloud& pc_1, const struct IPlotter::plot_info& pc_info_1) const;
        void plot(const PointCloud& pc) const;
        void plot(const PointCloud& pc, const struct IPlotter::plot_info& pc_info) const;
        void plot(const PointCloud& pc_0, const PointCloud& pc_1) const;
        void plot(const PointCloud& pc_0, const PointCloud& pc_1, const PointCloud& pc_2) const;
        void plot(const PointCloud& pc_0, const struct IPlotter::plot_info& pc_info_0,
                  const PointCloud& pc_1, const struct IPlotter::plot_info& pc_info_1) const;
        void plot(const PointCloud& pc_0, const struct IPlotter::plot_info& pc_info_0,
                  const PointCloud& pc_1, const struct IPlotter::plot_info& pc_info_1,
                  const PointCloud& pc_2, const struct IPlotter::plot_info& pc_info_2) const;
        void plot(const PointCloud& pc_0, const PointCloud& pc_1, const std::vector<uint32_t>& associate_list) const;
        void plot(const PointCloud& pc_0, const struct IPlotter::plot_info& pc_info_0, const PointCloud& pc_1, const struct IPlotter::plot_info& pc_info_1, const std::vector<uint32_t>& associate_list) const;
        void plot(const Pose2D& pose, const PointCloud& pc, const GridMap& grid_map) const;
        void plot(const Pose2D& pose, const struct IPlotter::plot_info& pose_info,
                  const PointCloud& pc, const struct IPlotter::plot_info& pc_info,
                  const GridMap& grid_map, const struct IPlotter::plot_info& map_info) const;

    protected:
        FILE *fd;
        void input_associates(const PointCloud& cur_pc, const PointCloud& ref_pc,
            const std::vector<uint32_t>& associate_list, const char *data_var) const;
        void input_normal(const PointCloud& pc, const char *data_var) const;
        void input_points(const PointCloud& pc, const char *data_var) const;
        void input_pose(const Pose2D& pose, const char *data_var) const;
        double to_point_size(double pt_size) const;
        double to_line_width(double line_width) const;
        uint32_t to_point_type(IPlotter::point_type pt_type) const;
        uint32_t to_color(IPlotter::color color) const;

        static constexpr double default_pt_size = 0.7f;
        static constexpr enum point_type default_pt_type = IPlotter::point_type::round_fill;
        static constexpr enum color default_color_0 = IPlotter::color::green;
        static constexpr enum color default_color_1 = IPlotter::color::purple;
        static constexpr enum color default_color_2 = IPlotter::color::red;
        static constexpr double default_line_width = 5.0f;

    private:
};
