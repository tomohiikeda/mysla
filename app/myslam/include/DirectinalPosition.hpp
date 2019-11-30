#pragma once
#include <iostream>

class DirectionalPosition{
    public:
        double direction;
        double x;
        double y;

        void print(void) {
            std::cout << x << ", " << y << ", " << direction << std::endl;
        }
        void move_to(DirectionalPosition move){
            direction += move.direction;
            x += move.x;
            y += move.y;
        }

    protected:
        
    private:
};
