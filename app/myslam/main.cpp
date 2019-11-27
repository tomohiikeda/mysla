/*
 * main.cpp
 */

#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include "GnuplotPlotter.hpp"
#include "Lidar.hpp"
#include "Odometer.hpp"
#include "Slam.hpp"

/**
 * @brief Ctrl+Cを押されたときのハンドラ
 */

bool ctrl_c_pressed;
static void ctrlc(int){ ctrl_c_pressed = true; }

/**
 * @brief メイン関数
 */
int main(int argc, const char *argv[])
{
    Lidar lidar;
    Odometer odometer;
    GnuplotPlotter plotter;
    Slam slam(&lidar, &odometer, &plotter);

    if (slam.init() == false) {
        return EXIT_FAILURE;
    }

    signal(SIGINT, ctrlc);

    if (slam.start() == false) {
        return EXIT_SUCCESS;
    }

    while (ctrl_c_pressed == false) {
        sleep(1);
    }

    slam.stop();

    return EXIT_SUCCESS;
}
