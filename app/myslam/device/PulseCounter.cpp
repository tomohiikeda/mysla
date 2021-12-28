#include "PulseCounter.hpp"
#include <fcntl.h>
#include <unistd.h>
#include <sys/types.h>
#include <errno.h>
#include <stdio.h>
#include <string>

bool PulseCounter::init(void)
{
    this->fd[0] = open("/dev/rtcounter_l1", O_RDWR);
    if (!this->fd[0]) {
        printf("failed to open /dev/rtcounter_l1\n");
        goto err_0;
    }

    this->fd[1] = open("/dev/rtcounter_r1", O_RDWR);
    if (!this->fd[1]) {
        printf("failed to open /dev/rtcounter_r1\n");
        goto err_1;
    }
    return true;

err_1:
    close(this->fd[0]);
err_0:
    return false;
}

bool PulseCounter::start(void)
{
    return true;
}

void PulseCounter::stop(void)
{
    for (int i=0; i<2; i++) {
        if (this->fd[i]) {
            close(this->fd[i]);
            this->fd[i] = 0;
        }
    }
}

bool PulseCounter::get_odometory(int16_t *od_l, int16_t *od_r)
{
    if (!fd[0] || !fd[1])
        return false;

    char buf_l[10] = {0};
    char buf_r[10] = {0};
    ssize_t rsize_l = read(this->fd[0], buf_l, sizeof(buf_l));
    ssize_t rsize_r = read(this->fd[1], buf_r, sizeof(buf_r));
    *od_l = atoi(buf_l);
    *od_r = atoi(buf_r);

    char cl[10] = {0};
    ssize_t wsize_l = write(this->fd[0], cl, sizeof(cl));
    ssize_t wsize_r = write(this->fd[1], cl, sizeof(cl));
    fsync(this->fd[0]);
    fsync(this->fd[1]);

    return true;

}    
