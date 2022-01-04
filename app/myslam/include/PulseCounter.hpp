#pragma once

#include "Common.hpp"
#include "IOdometer.hpp"

class PulseCounter : public IOdometer {
    public:
        bool init(void);
        void deinit(void);
        bool get_odometory(odometory_t *odom);
        bool get_odometory(int16_t *left, int16_t *right);

    protected:
        int fd[2];

    private:
};
