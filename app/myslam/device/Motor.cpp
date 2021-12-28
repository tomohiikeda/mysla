#include "Motor.hpp"
#include "RobotConst.hpp"
#include <string.h>

bool Motor::init(void)
{
    this->fd[0] = fopen("/dev/rtmotor_raw_r0", "w");
    if (!this->fd[0]) {
        printf("failed to open /dev/rtmotor_raw_r0\n");
        goto err_1;
    }

    this->fd[1] = fopen("/dev/rtmotor_raw_l0", "w");
    if (!this->fd[1]) {
        printf("failed to open /dev/rtmotor_raw_r0\n");
        goto err_0;
    }
    return true;

err_0:
    fclose(this->fd[0]);
err_1:
    return false;
}

void Motor::deinit(void)
{
    for (int i=0; i<2; i++) {
        if (this->fd[i]) {
            fclose(this->fd[i]);
            this->fd[i] = nullptr;
        }
    }
}

void Motor::move_front(double mm, double speed)
{
    if (this->fd[0] == nullptr || this->fd[1] == nullptr)
        return;

    //uint32_t freq = speed_to_freq(mm, speed);
    //move_front(freq);
}

void Motor::move_front(double speed)
{
    if (this->fd[0] == nullptr || this->fd[1] == nullptr)
        return;

    int16_t freq = speed_to_freq(speed);
    printf("freq=%d\n", freq);
    this->write_(freq, freq);
}

void Motor::move_front(int16_t freq)
{
    if (this->fd[0] == nullptr || this->fd[1] == nullptr)
        return;

    this->write_(freq, freq);
}

void Motor::turn_left(double rad, double speed)
{
}

void Motor::turn_left(int16_t freq)
{
    if (this->fd[0] == nullptr || this->fd[1] == nullptr)
        return;

    this->write_(-freq, freq);
}

void Motor::set_freq(int16_t freq_l, int16_t freq_r)
{
    this->write_(freq_l, freq_r);
}

void Motor::write_(int16_t freq_l, int16_t freq_r)
{
    char wbuf_l[10];
    char wbuf_r[10];

    sprintf(wbuf_l, "%d\n", freq_l);
    sprintf(wbuf_r, "%d\n", freq_r);
    size_t wrote_sz_l = fwrite(wbuf_l, strlen(wbuf_l), 1, this->fd[1]);
    size_t wrote_sz_r = fwrite(wbuf_r, strlen(wbuf_r), 1, this->fd[0]);
    
    if (!wrote_sz_l || !wrote_sz_r) {
        printf("failed to write to motor\n");
        this->deinit();
        return;
    }
    
    fflush(this->fd[1]);
    fflush(this->fd[0]);
}

int16_t Motor::speed_to_freq(double speed)
{
    return (PULSE_COUNT_PER_CYCLE * speed) / (2 * M_PI * WHEEL_RADIUS);
}