#pragma once

#include "IOdometer.hpp"
#include <stdio.h>

class PulseCounter : public IOdometer{
    public:
        bool init(void);
        bool start(void);
        void stop(void);
        bool get_odometory(int16_t *od_l, int16_t *od_r);

    protected:
        int fd[2];

    private:
};
