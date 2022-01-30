/*
 * main.cpp
 */
#include "Common.hpp"
#include "GnuplotPlotter.hpp"
#include "Lidar.hpp"
#include "PulseCounter.hpp"
#include "Slam.hpp"
#include "ScanMatcher.hpp"
#include "RemoteControl.hpp"
#include "Motor.hpp"
#include "PoseEstimator.hpp"
#include "GridMap.hpp"

static int slam_main(int argc, const char *argv[]);
static int save_main(int argc, const char *argv[]);
static int scan_matching_main(int argc, const char *argv[]);
static int scan_plot_main(int argc, const char *argv[]);
static int remocon_main(int argc, const char *argv[]);
static int debug_main(int argc, const char *argv[]);
static int usage_main(int argc, const char *argv[]);

struct argstr_func {
    const char *string;
    const int argnum;
    int (*func)(int argc, const char *argv[]);
    const char *desc;
};

static const struct argstr_func main_func_table[] = {
    { "slam",           5,  slam_main,          "slam"},
    { "slam_debug",     5,  slam_main,          "slam"},
    { "save",           3,  save_main,          "save slam data"},
    { "scan_plot",      1,  scan_plot_main,     "scan" },
    { "remocon",        1,  remocon_main,       "remocon" },
    { "matching",       4,  scan_matching_main, "scan matching" },
    { "debug",          1,  debug_main,         "debug" },
    { "usage",          1,  usage_main,         "usage" },
};
static constexpr int table_size = sizeof(main_func_table) / sizeof(argstr_func);

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
 * @param argv[1] "slam" or "slam_debug"
 * @param argv[2] mode "offline" or "online"
 * @param argv[3] ディレクトリ名
 * @param argv[4] 開始インデックス "-"にすると0
 * @param argv[5] 終了インデックス "-"にするとファイルがあるところまで
 * @return int
 */
static int slam_main(int argc, const char *argv[])
{
    Lidar lidar;
    PulseCounter pulse_sensor;
    std::string dirname(argv[3]);
    uint32_t start_index = atoi(argv[4]);
    uint32_t end_index = atoi(argv[5]);
    DataRetriever::slam_mode_e mode = strncmp(argv[2], "offline", 7) ?
                            DataRetriever::online_mode : DataRetriever::offline_mode;

    DataRetriever retriever(mode, lidar, pulse_sensor, dirname, start_index, end_index);
    GnuplotPlotter plotter;
    Slam *slam = new Slam(plotter, retriever);

    if (!strncmp(argv[1], "slam_debug", 10))
        slam->debug_on();

    Motor motor;
    RemoteControl remocon(motor);

    signal(SIGINT, ctrlc);

    if (slam->init() == false)
        return EXIT_FAILURE;

    if (slam->start() == false)
        return EXIT_FAILURE;

    if (motor.init() == false)
        return EXIT_FAILURE;

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
 * @param argv[3] Interval(sec)
 * @return int
 */
static int save_main(int argc, const char *argv[])
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

    PulseCounter pulse_sensor;
    if (pulse_sensor.init() == false)
        return EXIT_FAILURE;

    signal(SIGINT, ctrlc);

    uint32_t save_index = 0;
    while (ctrl_c_pressed == false) {
        SlamData slam_data;
        char filename[50] = {0};
        sprintf(filename, "%s/pt_%04d.txt", argv[2], save_index);
        lidar.get_point_cloud(slam_data.pc());
        pulse_sensor.get_odometory(slam_data.odometory());
        slam_data.save_to_file(filename);
        sleep(std::stod(argv[3]));
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
static int scan_matching_main(int argc, const char *argv[])
{
    GnuplotPlotter plotter;
    ScanMatcher scan_matcher(plotter, true);

    if (plotter.open() == false)
        return EXIT_FAILURE;

    int start = atoi(argv[3]);
    int end = atoi(argv[4]);
    Pose2D cur_pose(0, 0, 0);

    for (int i=start; i<end; i++) {
        printf("//-------------------------------------------------------\n");
        printf("//  Index = %d (%fmm, %fmm, %fdeg)\n", i, cur_pose.x, cur_pose.y, to_degree(cur_pose.direction));
        printf("//-------------------------------------------------------\n");
        SlamData ref_data;
        SlamData cur_data;
        char ref_filename[30];
        char cur_filename[30];
        sprintf(ref_filename, "%s/pt_%04d.txt", argv[2], i);
        sprintf(cur_filename, "%s/pt_%04d.txt", argv[2], i+1);
        ref_data.load_from_file(ref_filename);
        cur_data.load_from_file(cur_filename);
        ref_data.pc()->analyse_points();
        cur_data.pc()->analyse_points();

        Movement2D movement = scan_matcher.do_scan_matching(cur_data.pc(), ref_data.pc(), 2.0f);
        cur_pose.move(movement);
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
static int scan_plot_main(int argc, const char *argv[])
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
        plotter.plot(pc);
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
static int remocon_main(int argc, const char *argv[])
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

/**
 * @brief スキャンマッチングだけを行うモード
 *
 * @param argc
 * @param argv
 * @return int
 */
static int debug_main(int argc, const char *argv[])
{
    return EXIT_SUCCESS;
}

static int usage_main(int argc, const char *argv[])
{
    for (int i=0; i<table_size; i++) {
        printf("%02d %s %s\n", i, main_func_table[i].string, main_func_table[i].desc);
    }
    return EXIT_SUCCESS;
}

/**
 * @brief メイン関数
 */
int main(int argc, const char *argv[])
{
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
