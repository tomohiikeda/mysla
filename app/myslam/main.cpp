/*
 * main.cpp
 */

#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include "GnuplotPlotter.hpp"
#include "Lidar.hpp"
#include "PulseSensor.hpp"
#include "Slam.hpp"
#include "ScanMatcher.hpp"


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
    Lidar lidar;
    PulseSensor pulse_sensor;
    GnuplotPlotter plotter;
    Slam slam(&lidar, &pulse_sensor, &plotter);

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
#if 0

int main(int argc, const char *argv[])
{
    GnuplotPlotter *plotter = new GnuplotPlotter();
    PointCloud cur_scan;
    PointCloud ref_scan;
    ScanMatcher *scan_matcher = new ScanMatcher();
    plotter->open();

    ref_scan.load_from_file("pt_1.txt");
    cur_scan.load_from_file("pt_3.txt");
    scan_matcher->set_debug_plotter(plotter);
    scan_matcher->set_reference_scan(&ref_scan);
    scan_matcher->set_current_scan(&cur_scan);

    scan_matcher->do_scan_matching();

    while (ctrl_c_pressed == false) {
        sleep(1);
    }

    plotter->close();
    delete plotter;
    delete scan_matcher;
}
#endif
