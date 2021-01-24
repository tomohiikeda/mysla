#include "PulseCounter.hpp"

bool PulseCounter::init(void)
{
    this->fd_r = fopen("/dev/rtcounter_r0", "r");
    if (!this->fd_r) {
        printf("failed to open /dev/rtcounter_r0\n");
        goto err_0;
    }
    this->fd_l = fopen("/dev/rtcounter_l0", "r");
    if (!this->fd_l) {
        printf("failed to open /dev/rtcounter_l0\n");
        goto err_1;
    }

    return true;

err_1:
    fclose(fd_r);
err_0:
    return false;
}

bool PulseCounter::start(void)
{
    return true;
}

void PulseCounter::stop(void)
{
    if (this->fd_r)
        fclose(fd_r);
    
    if (this->fd_l)
        fclose(fd_l);
}

void PulseCounter::get_odometory(double *od_r, double *od_l)
{
    if (!fd_r || !fd_l)
        return;

    char buf[100];
    fseek(fd_r, 0, SEEK_SET);
    fgets(buf, sizeof(buf), fd_r);
    printf("%s\n", buf);
}    
