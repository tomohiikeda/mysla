#include "PulseSensor.hpp"

bool PulseSensor::init(void)
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

bool PulseSensor::start(void)
{
    return true;
}

void PulseSensor::stop(void)
{
    if (this->fd_r)
        fclose(fd_r);
    
    if (this->fd_l)
        fclose(fd_l);
}

void PulseSensor::get_odometory(double *od_r, double *od_l)
{
    if (!fd_r || !fd_l)
        return;

    char buf[100];
    fread(buf, sizeof(buf), 1, fd_r);
    printf("%s\n", buf);
}    
