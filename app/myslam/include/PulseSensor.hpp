#pragma once

#include "IOdometer.hpp"
#include <stdio.h>

class PulseSensor : public IOdometer{
    public:
        bool init(void);
        bool start(void);
        void stop(void);
        void get_odometory(double *od_r, double *od_l);

    protected:
        FILE *fd_r;
        FILE *fd_l;

    private:
};
