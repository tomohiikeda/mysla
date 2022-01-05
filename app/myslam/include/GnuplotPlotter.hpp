#pragma once

#include "IPlotter.hpp"
#include "PointCloud.hpp"

class GnuplotPlotter: public IPlotter{
    public:
        GnuplotPlotter(void){};
        virtual ~GnuplotPlotter(void){};
        bool open(void);
        void close(void);
        void plot(const Pose2D pose) const;
        void plot(const Pose2D pose, const PointCloud* pc) const;
        void plot(const PointCloud* pc) const;
        void plot(const PointCloud* pc0, const PointCloud* pc1) const;
        void plot(const PointCloud *pc_0,
                  const PointCloud *pc_1,
                  const std::vector<uint32_t>& associate_list) const;
        void plot(const GridMap& grid_map) const;
        void plot(const Pose2D pose, const GridMap& grid_map) const;

    protected:
        FILE *fd;
        void input_associates(const PointCloud *cur_pc, const PointCloud *ref_pc,
            const std::vector<uint32_t>& associate_list, const char *data_var) const;
        void input_normal(const PointCloud *pc, const char *data_var) const;
        void input_points(const PointCloud *pc, const char *data_var) const;
        void input_pose(const Pose2D pose, const char *data_var) const;

        const int MIN_PLOT_X = -3000;
        const int MAX_PLOT_X =  3000;
        const int MIN_PLOT_Y = -3000;
        const int MAX_PLOT_Y =  3000;
        const double POINT_SIZE = 0.7f;

    private:
};
