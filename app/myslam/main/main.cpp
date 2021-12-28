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
 * 
 */
bool ctrl_c_pressed;
static void ctrlc(int)
{
    ctrl_c_pressed = true;
}

/**
 * @brief SLAM実行モード
 * 
 * @param argc 
 * @param argv 
 * @return int 
 */
int slam_main(int argc, const char *argv[])
{
    printf("run SLAM mode\n");

    Lidar lidar;
    PulseCounter pulse_sensor;
    GnuplotPlotter plotter;
    Slam *slam = new Slam(lidar, pulse_sensor, plotter);

    Motor motor;
    RemoteControl remocon(motor);

    if (slam->init() == false)
        return EXIT_FAILURE;

    signal(SIGINT, ctrlc);

    if (slam->start(Slam::slam_mode) == false) {
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

    slam->stop();
    remocon.deinit();
    motor.deinit();

    delete slam;

    return EXIT_SUCCESS;
}

/**
 * @brief DS4で動かして、スキャンを保存するモード
 * 
 * @param argc 
 * @param argv 
 * @return int 
 */
int save_main(int argc, const char *argv[])
{
    printf("run save mode\n");

    Lidar lidar;
    if (lidar.init() == false)
        return EXIT_FAILURE;

    if (lidar.start() == false)
        return EXIT_FAILURE;

    Motor motor;
    if (motor.init() == false)
        return EXIT_FAILURE;

    RemoteControl remocon(motor);
    if (remocon.init() == false)
        return EXIT_FAILURE;

    GnuplotPlotter plotter;
    if (plotter.open() == false)
        return EXIT_FAILURE;

    signal(SIGINT, ctrlc);

    uint32_t save_index = 0;
    while (ctrl_c_pressed == false) {
        PointCloud pc;
        char filename[10] = {0};
        sprintf(filename, "pt_%04d.txt", save_index);
        lidar.get_point_cloud(&pc);
        pc.save_to_file(filename);
        //plotter.plot(&pc);
        sleep(0.3);
        save_index++;
    }

    lidar.stop();
    plotter.close();
    motor.deinit();
    remocon.deinit();

    return EXIT_SUCCESS;
}

/**
 * @brief スキャンマッチングだけを行うモード
 * 
 * @param argc 
 * @param argv 
 * @return int 
 */
int scan_matching_main(int argc, const char *argv[])
{
    printf("run matching mode\n");

    GnuplotPlotter plotter;
    PointCloud cur_scan;
    PointCloud ref_scan;
    ScanMatcher scan_matcher(&plotter);
    plotter.open();

    ref_scan.load_from_file("ref_scan.dat");
    cur_scan.load_from_file("cur_scan.dat");
    
    ref_scan.debug_print();

    scan_matcher.set_reference_scan(&ref_scan);
    scan_matcher.set_current_scan(&cur_scan);
    scan_matcher.do_scan_matching();
    while (ctrl_c_pressed == false) {
        sleep(1);
    }

    plotter.close();

    return EXIT_SUCCESS;
}

/**
 * @brief Lidarでスキャンしたものをリアルタイムでプロットするモード
 * 
 * @param argc 
 * @param argv 
 * @return int 
 */
int scan_plot_main(int argc, const char *argv[])
{
    printf("run scan_plot mode\n");

    Lidar lidar;
    if (lidar.init() == false)
        return EXIT_FAILURE;

    if (lidar.start() == false)
        return EXIT_FAILURE;

    GnuplotPlotter plotter;
    if (plotter.open() == false)
        return EXIT_FAILURE;

    signal(SIGINT, ctrlc);

    while (ctrl_c_pressed == false) {
        PointCloud pc;
        lidar.get_point_cloud(&pc);
        plotter.plot(&pc);
    }

    lidar.stop();
    plotter.close();

    return EXIT_SUCCESS;
}

/**
 * @brief DS4で操作するモード
 * 
 * @param argc 
 * @param argv 
 * @return int 
 */
int remocon_main(int argc, const char *argv[])
{
    printf("run remocon mode\n");

    Motor motor;
    if (motor.init() == false)
        return EXIT_FAILURE;

    RemoteControl remocon(motor);
    if (remocon.init() == false)
        return EXIT_FAILURE;

    signal(SIGINT, ctrlc);

    while (ctrl_c_pressed == false) {
        sleep(1);
    }

    motor.deinit();
    remocon.deinit();

    return EXIT_SUCCESS;
}

struct argstr_func {
    const char *string;
    const int argnum;
    int (*func)(int argc, const char *argv[]);
};

static const struct argstr_func main_func_table[] = {
    { "save",       1, save_main },
    { "scan_plot",  1, scan_plot_main },
    { "remocon",    1, remocon_main },
    { "matching",   3, scan_matching_main },
};
static const int table_size = sizeof(main_func_table) / sizeof(argstr_func);

/**
 * @brief メイン関数
 */
int main(int argc, const char *argv[])
{
    if (argc == 1) {
        return slam_main(argc, argv);
    }

    for (int i=0; i<table_size; i++){
        if (!strcmp(argv[1], main_func_table[i].string)) {
            if (main_func_table[i].argnum != (argc - 1)) {
                printf ("Argument num is invalid.\n");
                return EXIT_FAILURE;
            } else {
                return main_func_table[i].func(argc, argv);
            }
        }
    }

    printf("Invalid Argument \"%s\"\n", argv[1]);
    return EXIT_FAILURE;
}
