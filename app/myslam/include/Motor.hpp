#pragma once

#include "IMotor.hpp"
#include <stdio.h>

class Motor : public IMotor{
    public:
        bool init(void);
        void deinit(void);
        void move_front(double mm, double speed);
        void turn_left(double rad, double speed);

    protected:
        FILE *fd;

    private:
};
