#include "Common.hpp"
#include <iostream>
#include <unistd.h>
#include <time.h>
#include <sys/time.h>
#include "Slam.hpp"
#include "PoseEstimator.hpp"

/**
 * @brief SLAM初期化
 */
bool Slam::init(void)
{
    if(sensor.init() == false)
        return false;

    if(odometer.init() == false)
        return false;

    return true;
}


/**
 * @brief SLAM開始
 */
bool Slam::start(void)
{
    if(sensor.start() == false)
        return false;

    if(odometer.start() == false)
        return false;

    if(plotter.open() == false)
        return false;

    int err = pthread_create(&slam_thread, NULL, Slam::thread_entry, this);
    if(err){
        std::cout << "failed to pthread_create errno" << err << std::endl;
        return false;
    }
    return true;
}


/**
 * @brief SLAM停止
 */
void Slam::stop(void)
{
    running = false;
    pthread_join(slam_thread, NULL);
    sensor.stop();
    odometer.stop();
    plotter.close();
}


/**
 * @brief SLAMの実行スレッド
 */
void Slam::process_loop(void)
{
    PointCloud cur_pc;
    const double control_period = 0.1f;
    //ScanMatcher scan_matcher(&this->plotter);
    ScanMatcher scan_matcher(NULL);
    PoseEstimator pose_estimator(control_period, scan_matcher);
    struct timeval timeval;
    uint32_t loop_num = 0;

    this->running = true;

    // ずっとループ
    while (this->running == true) {

        gettimeofday(&timeval, NULL);
        //printf("%d\n",timeval.tv_usec);

        // Odometryを取得
        int16_t od_l, od_r;
        if (odometer.get_odometory(&od_l, &od_r) == false) {
            this->running = false;
            return;
        }

        // 現在のScanを取得
        if (sensor.get_point_cloud(&cur_pc) == false) {
            this->running = false;
            return;
        }

        // 現在位置の推定
        Pose2D cur_pose = pose_estimator.estimate_position(od_l, od_r, &cur_pc, this->glid_map);

        // ワールドマップを更新
        cur_pc.move(cur_pose);
        grow_world_map(&cur_pc);

        // 推定位置を表示
        cur_pose.print(loop_num);
        plotter.plot(cur_pose, this->glid_map);

        loop_num++;

        usleep(control_period * 1000 * 1000);
    }
    return;
}

/**
 * @brief 現在位置と測定点群からワールドマップを更新する
 */
void Slam::update_world_map(const PointCloud *cur_pc)
{
}

/**
 * @brief ワールドマップに現在点群を足して成長させる。
 * @param cur_pc 成長させる点群
 */
void Slam::grow_world_map(const PointCloud *cur_pc)
{
    //this->world_map.add(cur_pc);
    this->glid_map.set_points(cur_pc);
    return;
}

/**
 * @brief ワールドマップと測定点群から現在位置を推定する
 */
void Slam::estimate_cur_pose(const PointCloud *cur_pc)
{
    //cur_pos.print();
}

/**
 * @brief スレッドエントリ関数
 */
void *Slam::thread_entry(void *arg)
{
    ((Slam*)arg)->process_loop();
    return NULL;
}