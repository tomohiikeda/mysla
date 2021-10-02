#pragma once

#include "IOdometer.hpp"
#include <stdio.h>

class PulseCounter : public IOdometer{
    public:
        bool init(void);
        bool start(void);
        void stop(void);
        bool get_odometory(double *od_r, double *od_l);

    protected:
        int fd[2];

    private:
};
