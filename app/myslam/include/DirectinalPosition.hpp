#pragma once

class DirectionalPosition{
    public:
        double direction;
        double x;
        double y;

        void print(void) { printf("(%f, %f, %f)\n", x, y, direction); }
        void move_to(DirectionalPosition move){
            direction += move.direction;
            x += move.x;
            y += move.y;
        }

    protected:
        
    private:
};
