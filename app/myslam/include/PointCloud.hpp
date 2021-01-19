#pragma once

#include <stdint.h>
#include <vector>
#include <cmath>
#include "Point.hpp"
#include <fstream>
#include <iostream>
#include <sstream>
#include "Pose2D.hpp"
#include "Vector2D.hpp"

class PointCloud{
    public:
        void add(Point point);
        void add(const PointCloud *pc);
        const Point& at(int x) const;
        size_t size(void) const;
        void clear(void);
        void copy_to(PointCloud& to) const;
        void translate(double x, double y);
        void rotate(double radian);
        void move(Pose2D movement);
        void save_to_file(const char *filename) const;
        void load_from_file(const char *filename);
        void analyse_points(void);
        void debug_print(void);

    protected:
        std::vector<Point> points;
        bool calculate_normal(Vector2D& normal, int idx, const Point& pt, int dir);

    private:
};
