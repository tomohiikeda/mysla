#include "Motor.hpp"
#include <string.h>

bool Motor::init(void)
{
    this->fd[0] = fopen("/dev/rtmotor_raw_r0", "w");
    if (!this->fd[0]) {
        printf("failed to open /dev/rtmotor_raw_r0\n");
        goto err_1;
    }

    this->fd[1] = fopen("/dev/rtmotor_raw_l0", "w");
    if (!this->fd[0]) {
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

void Motor::move_front(int16_t freq)
{
    if (this->fd[0] == nullptr || this->fd[1] == nullptr)
        return;

    char wbuf_l[10];
    char wbuf_r[10];

    sprintf(wbuf_l, "%d\n", freq);
    sprintf(wbuf_r, "%d\n", freq);
    size_t wrote_sz_l = fwrite(wbuf_l, strlen(wbuf_l), 1, this->fd[1]);
    size_t wrote_sz_r = fwrite(wbuf_r, strlen(wbuf_r), 1, this->fd[0]);
    fflush(this->fd[0]);
    fflush(this->fd[1]);
}

void Motor::turn_left(double rad, double speed)
{
}

void Motor::turn_left(int16_t freq)
{
    if (this->fd[0] == nullptr || this->fd[1] == nullptr)
        return;

    char wbuf_l[10];
    char wbuf_r[10];
    sprintf(wbuf_l, "%d\n", -freq);
    sprintf(wbuf_r, "%d\n", freq);
    size_t wrote_sz_l = fwrite(wbuf_l, strlen(wbuf_l), 1, this->fd[1]);
    size_t wrote_sz_r = fwrite(wbuf_r, strlen(wbuf_r), 1, this->fd[0]);
    fflush(this->fd[1]);
    fflush(this->fd[0]);
}
