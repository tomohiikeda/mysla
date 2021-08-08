#include "PulseCounter.hpp"
#include <fcntl.h>
#include <unistd.h>
#include <sys/types.h>
#include <errno.h>

bool PulseCounter::init(void)
{
    this->fd_r = open("/dev/rtcounter_r1", O_RDONLY);
    if (!this->fd_r) {
        printf("failed to open /dev/rtcounter_r0\n");
        goto err_0;
    }
    return true;

err_1:
    close(this->fd_r);
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
        close(fd_r);
    
    if (this->fd_l)
        close(fd_l);
}

bool PulseCounter::get_odometory(double *od_r, double *od_l)
{

//    if (!fd_r || !fd_l)
//        return;

    char buf[10] = {0};
    off_t ofs = lseek(this->fd_r, 0, SEEK_SET);
    ssize_t rsize = read(this->fd_r, buf, sizeof(buf));
    //printf("ofs=%d, errno=%d, rsize=%d\n", ofs, errno, rsize);
    //printf("%s\n", buf);
    return true;

}    
