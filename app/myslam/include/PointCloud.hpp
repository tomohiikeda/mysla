#pragma once

#include <stdint.h>
#include <vector>
#include "Point.hpp"

class PointCloud{
    public:
        void add(Point point){points.push_back(point);}

    protected:
        std::vector<Point> points;

    private:
};
