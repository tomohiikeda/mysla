#pragma once

#include <stdint.h>
#include <vector>
#include "Point.hpp"

class PointCloud{
    public:
        void add(Point point){ points.push_back(point); }
        Point& at(int x){ return points.at(x); }
        size_t size(void){ return points.size(); }
        void clear(void) { points.clear(); }
        void copy_to(PointCloud& to){
            to.clear();
            for (size_t i=0; i<points.size(); i++)
                to.add(points.at(i));
        }

    protected:
        std::vector<Point> points;

    private:
};
