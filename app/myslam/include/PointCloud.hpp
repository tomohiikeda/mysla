#pragma once

#include <stdint.h>
#include <vector>
#include "Point.hpp"

class PointCloud{
    public:
        void add(Point point){points.push_back(point);}
        Point& at(int x){return points.at(x);}
        size_t size(void){return points.size();}

    protected:
        std::vector<Point> points;

    private:
};
