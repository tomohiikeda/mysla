#pragma once
#include <cmath>
#include "Vector2D.hpp"
#include "Movement2D.hpp"

typedef enum {
    PT_LINE,
    PT_CORNER,
    PT_ISOLATE,
    PT_NA,
} PointType;

class Point{
    public:
        Point();
        Point(double x, double y);
        double x = 0;
        double y = 0;
        Vector2D normal;
        PointType type;
        double distance_to(const Point& p) const;
        double vertical_distance_to(const Point& p) const ;
        void move(const Movement2D& movement);

    protected:
        

    private:
};
