#pragma once

#include <stdint.h>
#include <vector>
#include <cmath>
#include "Point.hpp"

class PointCloud{
    public:
        void add(Point point){points.push_back(point); }
        
        void add(const PointCloud& pc){
            for (uint32_t i=0; i<pc.size(); i++){
                points.push_back(pc.at(i));
            }
        }
        
        const Point& at(int x) const { return points.at(x); }
        size_t size(void) const { return points.size(); }
        void clear(void) { points.clear(); }
        
        void copy_to(PointCloud& to) const {
            to.clear();
            for (size_t i=0; i<points.size(); i++)
                to.add(points.at(i));
        }
        
        void move(double x, double y){
            for (size_t i=0; i<points.size(); i++) {
                points.at(i).x += x;
                points.at(i).y += x;
            }
        }
        
        void rotate(double radian){
            for (size_t i=0; i<points.size(); i++) {
                double x = points.at(i).x;
                double y = points.at(i).y;
                points.at(i).x  = x * std::cos(radian) - y * std::sin(radian);
                points.at(i).y  = x * std::sin(radian) + y * std::cos(radian);
            }
        }

        void optimize(void){
        }

    protected:
        std::vector<Point> points;

    private:
};
