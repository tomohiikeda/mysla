/*
 * main.cpp
 */

#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include "GnuplotPlotter.hpp"
#include "Lidar.hpp"


/**
 * @brief Ctrl+Cを押されたときのハンドラ
 */

bool ctrl_c_pressed;
static void ctrlc(int)
{
    ctrl_c_pressed = true;
}

/**
 * @brief メイン関数
 */
int main(int argc, const char *argv[])
{
    GnuplotPlotter plotter;
    Lidar lidar(&plotter);

    if (lidar.init("/dev/ttyUSB0", 115200) == false) {
        return EXIT_FAILURE;
    }

    signal(SIGINT, ctrlc);

    bool isSuccess = lidar.start();
    if(isSuccess == false){
        return EXIT_SUCCESS;
    }

    while (ctrl_c_pressed == false) {
        sleep(1);
    }

    lidar.stop();

    return EXIT_SUCCESS;
}
