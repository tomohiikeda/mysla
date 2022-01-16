#include "Point.hpp"

Point::Point(void)
{
    this->x = 0;
    this->y = 0;
    this->normal = {0, 0};
    this->type = PT_ISOLATE;
}

Point::Point(double x, double y):x(x),y(y)
{
    this->normal = {0, 0};
    this->type = PT_ISOLATE;
}

double Point::distance_to(const Point& p) const {
    return std::sqrt(std::pow(x - p.x, 2) + std::pow(y - p.y, 2));
}

double Point::vertical_distance_to(const Point& p) const {
    double d = (this->x - p.x) * p.normal.x + (this->y - p.y)* p.normal.y;
    return d * d;
}
