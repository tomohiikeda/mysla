#pragma once
#include <iostream>

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
            printf("[%05d]%lf, %lf, %lf\n", num, x, y, direction);
        }
        void move_to(const Pose2D& move){
            direction += move.direction;
            x += move.x;
            y += move.y;
        }

    protected:

    private:
};
