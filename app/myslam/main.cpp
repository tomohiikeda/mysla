/*
 * main.cpp
 */

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>
#include "Lidar.hpp"
#include "GnuplotPlotter.hpp"

#define _countof(_Array) (int)(sizeof(_Array) / sizeof(_Array[0]))

static inline void delay(_word_size_t ms){
    while (ms>=1000){
        usleep(1000*1000);
        ms-=1000;
    };
    if (ms!=0)
        usleep(ms*1000);
}

/**
 * @brief Ctrl+Cを押されたときのハンドラ
 */
bool ctrl_c_pressed;
void ctrlc(int)
{
    ctrl_c_pressed = true;
}


/**
 * @brief メイン関数
 */
int main(int argc, const char * argv[])
{
    GnuplotPlotter plotter;

    Lidar lidar(plotter);
    
    bool ret = lidar.init("/dev/ttyUSB0", 115200);
    if(ret == false){
        return EXIT_FAILURE;
    }

    signal(SIGINT, ctrlc);

    lidar.start();

    while(ctrl_c_pressed == false){
        sleep(1);
    }

    lidar.stop();

    return EXIT_SUCCESS;
}

