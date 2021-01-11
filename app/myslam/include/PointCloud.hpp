#pragma once

#include <stdint.h>
#include <vector>
#include <cmath>
#include "Point.hpp"
#include <fstream>
#include <iostream>
#include <sstream>
#include "Pose2D.hpp"

class PointCloud{
    public:
        void add(Point point){points.push_back(point); }

        void add(const PointCloud *pc){
            for (uint32_t i=0; i<pc->size(); i++){
                points.push_back(pc->at(i));
            }
        }

        const Point& at(int x) const {
            return points.at(x);
        }

        size_t size(void) const {
            return points.size();
        }

        void clear(void) {
            points.clear();
        }

        void copy_to(PointCloud& to) const {
            to.clear();
            for (size_t i=0; i<points.size(); i++)
                to.add(points.at(i));
        }

        void translate(double x, double y){
            for (size_t i=0; i<points.size(); i++) {
                points.at(i).x += x;
                points.at(i).y += x;
            }
        }

        void rotate(double radian) {
            for (size_t i=0; i<points.size(); i++) {
                double x = points.at(i).x;
                double y = points.at(i).y;
                points.at(i).x  = x * std::cos(radian) - y * std::sin(radian);
                points.at(i).y  = x * std::sin(radian) + y * std::cos(radian);
            }
        }

        void move(Pose2D movement){
            this->rotate(movement.direction);
            this->translate(movement.x, movement.y);
        }

        void save_to_file(const char *filename) const
        {
            std::ofstream ofs(filename);
            for (size_t i=0; i<points.size(); i++){
                ofs << points.at(i).x << " " << points.at(i).y << std::endl;
            }
            ofs.close();
        }

        void load_from_file(const char *filename)
        {
            std::ifstream ifs(filename);
            if (!ifs)
                return;

            std::string line;
            while (getline(ifs, line)) {
                std::stringstream ss;
                std::string s, x, y;
                ss << line;
                getline(ss, x, ' ');
                getline(ss, y, ' ');
                Point p(stod(x), stod(y));
                this->add(p);
            }
            ifs.close();
        }

    protected:
        std::vector<Point> points;

    private:
};
