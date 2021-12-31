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
#include "PoseEstimator.hpp"

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
 * @param argv[2] 保存するディレクトリ名
 * @return int 
 */
int save_main(int argc, const char *argv[])
{
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
        char filename[30] = {0};
        sprintf(filename, "%s/pt_%04d.txt", argv[2], save_index);
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
 * @param argv[2] データディレクトリ名
 * @param argv[3] 開始インデックス
 * @param argv[4] 最終インデックス
 * @return int 
 */
int scan_matching_main(int argc, const char *argv[])
{
    GnuplotPlotter plotter;
    ScanMatcher scan_matcher(&plotter);

    if (plotter.open() == false)
        return EXIT_FAILURE;

    int start = atoi(argv[3]);
    int end = atoi(argv[4]);
    Pose2D cur_pose;

    for (int i=start; i<end; i++) {
        printf("//-------------------------------------------------------\n");
        printf("//  Index = %d (%fmm, %fmm, %fdeg)\n", i, cur_pose.x, cur_pose.y, to_degree(cur_pose.direction));
        printf("//-------------------------------------------------------\n");
        PointCloud ref_scan;
        PointCloud cur_scan;
        char ref_filename[20];
        char cur_filename[20];
        sprintf(ref_filename, "%s/pt_%04d.txt", argv[2], i);
        sprintf(cur_filename, "%s/pt_%04d.txt", argv[2], i+1);
        ref_scan.load_from_file(ref_filename);
        cur_scan.load_from_file(cur_filename);
        ref_scan.analyse_points();

        Pose2D dev = scan_matcher.do_scan_matching(&cur_scan, &ref_scan, 0.5f);

        double theta = cur_pose.direction + dev.direction;
        double x = dev.x * cos(theta) - dev.y * sin(theta);
        double y = dev.x * sin(theta) + dev.y * cos(theta);
        Pose2D move_world(x, y, dev.direction);
        cur_pose.move_to(move_world);
    }

    printf("//-------------------------------------------------------\n");
    printf("//  Last Position (%fmm, %fmm, %fdeg)\n", cur_pose.x, cur_pose.y, to_degree(cur_pose.direction));
    printf("//-------------------------------------------------------\n");

    while (ctrl_c_pressed == false) {
        sleep(1);
    }

    plotter.close();

    return EXIT_SUCCESS;
}

/**
 * @brief 位置推定器のテスト
 * 
 * @param argc 
 * @param argv[2] データディレクトリ名
 * @param argv[3] 開始インデックス
 * @param argv[4] 最終インデックス
 * @return int 
 */
int pose_estimate_main(int argc, const char *argv[])
{
    GnuplotPlotter plotter;
    ScanMatcher scan_matcher;
    PointCloud init_scan;
    PoseEstimator pose_estimator(scan_matcher);
    Pose2D cur_pose;

    if (plotter.open() == false)
        return EXIT_FAILURE;

    char inifile[20];
    sprintf(inifile, "%s/pt_0000.txt", argv[2]);
    init_scan.load_from_file(inifile);

    int start = atoi(argv[3]);
    int end = atoi(argv[4]);

    for (int i=start; i<end; i++) {
        
        printf("//-------------------------------------------------------\n");
        printf("//  Index = %d (%fmm, %fmm, %fdeg)\n", i, cur_pose.x, cur_pose.y, to_degree(cur_pose.direction));
        printf("//-------------------------------------------------------\n");
        
        PointCloud cur_scan;
        char cur_filename[20];
        sprintf(cur_filename, "%s/pt_%04d.txt", argv[2], i+1);
        cur_scan.load_from_file(cur_filename);
        cur_scan.analyse_points();
        cur_pose = pose_estimator.estimate_position(&cur_scan);
        
        plotter.plot(cur_pose, &init_scan);
    }

    printf("//-------------------------------------------------------\n");
    printf("//  Last Position (%fmm, %fmm, %fdeg)\n", cur_pose.x, cur_pose.y, to_degree(cur_pose.direction));
    printf("//-------------------------------------------------------\n");

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
    { "save",           2,  save_main },
    { "scan_plot",      1,  scan_plot_main },
    { "remocon",        1,  remocon_main },
    { "matching",       4,  scan_matching_main },
    { "pose_estimate",  4,  pose_estimate_main },
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
                printf("run %s mode\n", argv[1]);
                return main_func_table[i].func(argc, argv);
            }
        }
    }

    printf("Invalid Argument \"%s\"\n", argv[1]);
    return EXIT_FAILURE;
}
