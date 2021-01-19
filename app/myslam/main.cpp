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
#include <string.h>

/**
 * @brief Ctrl+Cを押されたときのハンドラ
 */

bool ctrl_c_pressed;
static void ctrlc(int)
{
    ctrl_c_pressed = true;
}

int slam_main(int argc, const char *argv[])
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

int save_main(int argc, const char *argv[])
{
    Lidar lidar;
    if(lidar.init() == false)
        return false;

    if(lidar.start() == false)
        return EXIT_FAILURE;

    PointCloud cur_pc;
    for (int i=0; i<4; i++) {
        if (lidar.get_point_cloud(&cur_pc) == false) {
            goto exit;
        }
        char filename[20];
        snprintf(filename, sizeof(filename), "pt_%d.txt", i);
        cur_pc.save_to_file(filename);
        printf("%d\n", i);
        sleep(3);
    }

exit:
    lidar.stop();
    return EXIT_SUCCESS;
}

int scan_matching_main(int argc, const char *argv[])
{
    GnuplotPlotter *plotter = new GnuplotPlotter();
    PointCloud cur_scan;
    PointCloud ref_scan;
    ScanMatcher *scan_matcher = new ScanMatcher();
    plotter->open();

    ref_scan.load_from_file("pt_0.txt");
    cur_scan.load_from_file("pt_3.txt");
    
    ref_scan.analyse_points();
    ref_scan.debug_print();

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

/**
 * @brief メイン関数
 */
int main(int argc, const char *argv[])
{
    if (argc > 1) {
        if (!strcmp(argv[1], "save"))
            return save_main(argc, argv);
        else if (!strcmp(argv[1], "matching"))
            return scan_matching_main(argc, argv);
    } else {
        return slam_main(argc, argv);
    }
}
