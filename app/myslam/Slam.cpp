#include <stdio.h>
#include <iostream>
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
    //odometer->stop();
    plotter->close();
}


/**
 * @brief SLAMの実行スレッド
 */
void Slam::process_loop(void)
{
    PointCloud pre_pc;
    PointCloud cur_pc;
    
    running = true;

    if (sensor->get_point_cloud(&pre_pc) == false)
        return;

    pre_pc.copy_to(world_map);  // 初回
    
    // ずっとループ
    for (int loop_cnt=0; loop_cnt<4; loop_cnt++) {
    //while (running == true) {
        
        if (sensor->get_point_cloud(&cur_pc) == false) {
            running = false;
            return;
        }
        
        //update_world_map(cur_pc);
        //estimate_cur_pose(cur_pc);
        cur_pc.copy_to(pre_pc);
        plotter->plot(&cur_pc);

        char filename[20];
        snprintf(filename, sizeof(filename), "pt_%d.txt", loop_cnt);
        cur_pc.save_to_file(filename);

        printf("%d\n", loop_cnt);
        
        sleep(1);
    }
    return;
}

void Slam::wait_for_key(void) const
{
    char aaa;
    std::cin >> aaa;
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