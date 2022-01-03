#pragma once
#include <iostream>
#include "Util.hpp"

class Pose2D {
    public:
        Pose2D(void){ Pose2D(0, 0, 0); }
        Pose2D(double x, double y, double dir){
            this->x = x;
            this->y = y;
            this->direction = dir;
        }
        
        double x;
        double y;
        double direction;

        void print(void) {
            printf("%lf, %lf, %lf\n", x, y, direction);
        }
        void print(uint32_t num) {
            printf("[%05d]%lf, %lf, %lf(%lf degree)\n", num, x, y, direction, to_degree(direction));
        }
        void move_to(const Pose2D& move){
            this->direction += move.direction;
            this->x += move.x;
            this->y += move.y;
        }
        void set(double x, double y, double dir) {
            this->direction = dir;
            this->x = x;
            this->y = y;
        }

    protected:

    private:
};
