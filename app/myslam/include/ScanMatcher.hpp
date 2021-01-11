#pragma once

#include "PointCloud.hpp"
#include "Pose2D.hpp"
#include "IPlotter.hpp"

class ScanMatcher {
    public:
        ScanMatcher(void){};
        void set_current_scan(const PointCloud *pc);
        void set_reference_scan(const PointCloud *pc);
        Pose2D do_scan_matching(void) const;
        void set_debug_plotter(const IPlotter *plotter);
    
    private:
        const IPlotter *debug_plotter;
        const PointCloud *cur_scan;
        const PointCloud *ref_scan;
        Pose2D minimize_cost_pose(const PointCloud *scan,
                                  const PointCloud *ref_scan,
                                  const std::vector<uint32_t>& associate_list) const;
        void data_associate(const PointCloud *cur_scan,
                            const PointCloud *ref_scan,
                            std::vector<uint32_t>& associate_list) const;
        uint32_t find_nearest_index(const Point point, const PointCloud *pc) const;
        double cost_function(const PointCloud *cur_scan,
                             const PointCloud *ref_scan,
                             const std::vector<uint32_t>& associate_list) const;
        void plot_for_debug(const PointCloud *cur_scan,
                            const PointCloud *ref_scan,
                            const std::vector<uint32_t>& associate_list) const;

};
