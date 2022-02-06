#pragma once

#include "Common.hpp"
#include "GridMap.hpp"
#include "IPlotter.hpp"
#include "Trajectory.hpp"

class PathDetector {
    public:
        PathDetector(IPlotter& plotter, bool debug): plotter(plotter), debug_mode(debug) {};
        virtual ~PathDetector(void) {};
        void search_path(Trajectory& path, const GridMap& map,
                         const Point& start, const Point& goal, double speed = 1.0f) const;

    protected:
        const IPlotter& plotter;
        bool debug_mode;
        bool is_debug_mode(void) const { return debug_mode; }
};
