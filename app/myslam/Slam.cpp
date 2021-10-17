#include "Common.hpp"
#include <iostream>
#include <unistd.h>
#include "Slam.hpp"
#include "PoseEstimator.hpp"

/**
 * @brief SLAM初期化
 */
bool Slam::init(void)
{
    if(sensor->init() == false)
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
    if(sensor->start() == false)
        return false;

    if(odometer.start() == false)
        return false;

    if(plotter->open() == false)
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
    sensor->stop();
    odometer.stop();
    plotter->close();
}


/**
 * @brief SLAMの実行スレッド
 */
void Slam::process_loop(void)
{
    PointCloud pre_pc;
    PointCloud cur_pc;
    const double control_period = 0.1f;
    PoseEstimator pose_estimator(control_period);
    uint32_t cnt = 0;

    running = true;

    if (sensor->get_point_cloud(&pre_pc) == false)
        return;

    pre_pc.copy_to(world_map);  // 初回
    this->scan_matcher->set_reference_scan(&pre_pc);
    pre_pc.analyse_points();

    // ずっとループ
    while (running == true) {

        cnt++;

        int16_t od_l, od_r;
        odometer.get_odometory(&od_l, &od_r);
        Pose2D cur_pose = pose_estimator.get_estimated_position(od_l, od_r);
        //cur_pose.print();

        //plotter->plot_pose(cur_pose);
        
        if (sensor->get_point_cloud(&cur_pc) == false) {
            running = false;
            return;
        }
        /*
        this->scan_matcher->set_current_scan(&cur_pc);
        Pose2D movement = this->scan_matcher->do_scan_matching();
        cur_pc.move(movement);
        this->plotter->plot(&cur_pc, &pre_pc);
        */
        //cur_pc.copy_to(pre_pc);
        //this->scan_matcher->set_reference_scan(&pre_pc);
        
        
        //update_world_map(cur_pc);
        //estimate_cur_pose(cur_pc);
        //cur_pc.copy_to(pre_pc);
        plotter->plot(cur_pose, &cur_pc);

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