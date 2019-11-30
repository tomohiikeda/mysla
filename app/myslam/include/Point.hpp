#pragma once
#include <cmath>

class Point{
    public:
        Point(double x, double y):x(x),y(y){};
        double x;
        double y;
        double distance_to(const Point& p) const {
            return std::sqrt(std::pow(x - p.x, 2) + std::pow(y - p.y, 2));
        }

    protected:
        

    private:
};
