#include <stdio.h>
#include "Slam.hpp"

bool Slam::init(void)
{
    bool ret = this->sensor->init();
    return ret;
}

bool Slam::start(void)
{
    if(sensor->start() == false){
        return false;
    }

    //if(odometer->start() == false){
    //    return false;
    //}

    if(plotter->open() == false){
        return false;
    }

    int err = pthread_create(&slam_thread, NULL, Slam::process_loop, this);
    if(err){
        printf("failed to pthread_create errno=%d\n", err);
        return false;
    }
    running = true;
    return true;
}

void Slam::stop(void)
{
    running = false;
    pthread_join(slam_thread, NULL);
    sensor->stop();
    //odometer->stop();
    plotter->close();
}

/**
 * @brief SLAMの実行スレッド
 */
void *Slam::process_loop(void *arg)
{
    Slam *slam = (Slam*)arg;

    while(slam->running){
        
        PointCloud pc;
        if (slam->sensor->get_point_cloud(pc) == false) {
            slam->running = false;
            return NULL;
        }

        slam->plotter->plot(pc);
    }
    return NULL;
}
