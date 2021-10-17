/*
 * main.cpp
 */

#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include "GnuplotPlotter.hpp"
#include "Lidar.hpp"
#include "PulseCounter.hpp"
#include "Slam.hpp"
#include "ScanMatcher.hpp"
#include "RemoteControl.hpp"
#include <string.h>
#include "Motor.hpp"

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
    printf("run SLAM mode\n");

    Lidar lidar;
    PulseCounter pulse_sensor;
    GnuplotPlotter plotter;
    Slam slam(lidar, pulse_sensor, plotter);
    
    Motor motor;
    RemoteControl remocon(motor);

    

    if (slam.init() == false)
        return EXIT_FAILURE;

    signal(SIGINT, ctrlc);

    if (slam.start() == false) {
        printf("failed to start SLAM\n");
        return EXIT_FAILURE;
    }

    if (motor.init() == false) {
        printf("failed to motor init\n");
        return EXIT_FAILURE;
    }

    if (remocon.init() == false)
        return EXIT_FAILURE;

    while (ctrl_c_pressed == false)
        sleep(1);

    slam.stop();
    remocon.deinit();
    motor.deinit();

    return EXIT_SUCCESS;
}

int save_main(int argc, const char *argv[])
{
    printf("run save mode\n");

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
    printf("run matching mode\n");

    GnuplotPlotter *plotter = new GnuplotPlotter();
    PointCloud cur_scan;
    PointCloud ref_scan;
    ScanMatcher *scan_matcher = new ScanMatcher(true);
    plotter->open();

    ref_scan.load_from_file("ref_scan.dat");
    cur_scan.load_from_file("cur_scan.dat");
    
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

    return EXIT_SUCCESS;
}

struct argstr_func {
    const char *string;
    int (*func)(int argc, const char *argv[]);
};

static const struct argstr_func main_func_table[] = {
    { "save",       save_main },
    { "matching",   scan_matching_main },
};
static const int table_size = sizeof(main_func_table) / sizeof(argstr_func);

/**
 * @brief メイン関数
 */
int main(int argc, const char *argv[])
{
    if (argc > 2) {
        printf ("Argument is too much.");
        return EXIT_FAILURE;
    }

    if (argc == 1) {
        return slam_main(argc, argv);
    }

    if (argc == 2) {
        for (int i=0; i<table_size; i++){
            if (!strcmp(argv[1], main_func_table[i].string)) {
                return main_func_table[i].func(argc, argv);
            }
        }
    }

    printf("Invalid Argument \"%s\"\n", argv[1]);
    return EXIT_FAILURE;
}
