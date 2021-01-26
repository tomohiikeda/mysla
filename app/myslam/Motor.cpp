#include "Motor.hpp"

bool Motor::init(void)
{
    this->fd = fopen("/dev/rtmotor_raw_r0", "r");
    if (!this->fd) {
        printf("failed to open /dev/rtmotor_raw_r0\n");
        return false;
    }
    return true;
}

void Motor::deinit(void)
{
    if (this->fd) {
        fclose(this->fd);
        this->fd = nullptr;
    }
}

void Motor::move_front(double mm, double speed)
{
}

void Motor::turn_left(double rad, double speed)
{
}
