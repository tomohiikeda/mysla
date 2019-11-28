#include <stdio.h>
#include <unistd.h>
#include "Slam.hpp"

/**
 * @brief SLAM初期化
 */
bool Slam::init(void)
{
    if(sensor->init() == false)
        return false;

    if(odometer->init() == false)
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

    if(odometer->start() == false)
        return false;

    if(plotter->open() == false)
        return false;

    int err = pthread_create(&slam_thread, NULL, Slam::thread_entry, this);
    if(err){
        printf("failed to pthread_create errno=%d\n", err);
        return false;
    }
    running = true;
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
    //odometer->stop();
    plotter->close();
}


/**
 * @brief SLAMの実行スレッド
 */
void Slam::process_loop(void)
{
    PointCloud pre_pc;

    while(running){
        
        PointCloud cur_pc;

        if (sensor->get_point_cloud(cur_pc) == false) {
            running = false;
            return;
        }
        
        update_world_map(cur_pc);
        estimate_cur_pos(cur_pc);

        cur_pc.copy_to(pre_pc);
        
    }
    return;
}


/**
 * @brief 現在位置と測定点群からワールドマップを更新する
 */
void Slam::update_world_map(PointCloud& cur_pc)
{
    cur_pc.copy_to(world_map);
    plotter->plot(world_map);
}


/**
 * @brief ワールドマップと測定点群から現在位置を推定する
 */
void Slam::estimate_cur_pos(PointCloud& cur_pc)
{
    cur_pos.print();
}

/**
 * @brief スレッドエントリ関数
 */
void *Slam::thread_entry(void *arg)
{
    ((Slam*)arg)->process_loop();
    return NULL;
}