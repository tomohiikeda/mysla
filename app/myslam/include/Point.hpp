#pragma once
#include <cmath>
#include "Vector2D.hpp"

typedef enum {
    PT_LINE,
    PT_CORNER,
    PT_ISOLATE,
    PT_NA,
} PointType;

class Point{
    public:
        Point(double x, double y):x(x),y(y) 
        {
            this->normal = {0, 0};
            this->type = PT_ISOLATE;
        };
        double x;
        double y;
        Vector2D normal;
        PointType type;

        double distance_to(const Point& p) const {
            return std::sqrt(std::pow(x - p.x, 2) + std::pow(y - p.y, 2));
        }

        double vertical_distance_to(const Point& p) const {
            double d = (this->x - p.x) * p.normal.x + (this->y - p.y)* p.normal.y;
            return d * d;
        }

    protected:
        

    private:
};
