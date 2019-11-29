#pragma once

#include <stdint.h>
#include <vector>
#include "Point.hpp"

class PointCloud{
    public:
        void add(Point point){ points.push_back(point); }
        const Point& at(int x) const { return points.at(x); }
        size_t size(void) const { return points.size(); }
        void clear(void) { points.clear(); }
        void copy_to(PointCloud& to) const {
            to.clear();
            for (size_t i=0; i<points.size(); i++)
                to.add(points.at(i));
        }

    protected:
        std::vector<Point> points;

    private:
};
