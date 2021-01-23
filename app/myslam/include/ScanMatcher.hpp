#pragma once

#include "PointCloud.hpp"
#include "Pose2D.hpp"
#include "IPlotter.hpp"

class ScanMatcher {
    public:
        ScanMatcher(bool debug=false){
            this->debug = debug;
        };
        void set_current_scan(const PointCloud *pc);
        void set_reference_scan(const PointCloud *pc);
        Pose2D do_scan_matching(void) const;
        void set_debug_plotter(const IPlotter *plotter);
    
    private:
        enum cost_type {
            COST_SIMPLE,
            COST_VERTICAL,
        };
        bool debug = false;
        const IPlotter *debug_plotter;
        const PointCloud *cur_scan;
        const PointCloud *ref_scan;
        double differential(const PointCloud *scan,
                            const PointCloud *ref_scan,
                            const std::vector<uint32_t>& associate_list,
                            double ev,
                            double dd,
                            double kk,
                            enum cost_type cost_type) const;
        uint32_t is_matching_done(
                const double ev,
                const double pre_ev,
                double *ev_history,
                uint32_t history_num,
                enum cost_type cost_type) const;
        Pose2D steepest_descent(const PointCloud *scan,
                                const PointCloud *ref_scan,
                                const std::vector<uint32_t>& associate_list,
                                const enum cost_type cost_type) const;
        Pose2D full_search(const PointCloud *scan,
                           const PointCloud *ref_scan,
                           const std::vector<uint32_t>& associate_list) const;
        void data_associate(const PointCloud *cur_scan,
                            const PointCloud *ref_scan,
                            std::vector<uint32_t>& associate_list) const;
        uint32_t find_nearest_index(const Point point, const PointCloud *pc) const;
        double simple_distance(const PointCloud *cur_scan,
                               const PointCloud *ref_scan,
                               const std::vector<uint32_t>& associate_list) const;
        double vertical_distance(const PointCloud *cur_scan,
                                 const PointCloud *ref_scan,
                                 const std::vector<uint32_t>& associate_list) const;
        double cost_function(const PointCloud *cur_scan,
                             const PointCloud *ref_scan,
                             const std::vector<uint32_t>& associate_list,
                             const enum cost_type cost_type) const;
        void plot_for_debug(const PointCloud *cur_scan,
                            const PointCloud *ref_scan,
                            const std::vector<uint32_t>& associate_list) const;

};
