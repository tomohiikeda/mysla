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

        //wait_for_key();

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
void Slam::update_world_map(const PointCloud& cur_pc)
{
    double cost = calculate_cost(cur_pc);
    printf("cost=%6.2f\n", cost);

    cur_pc.copy_to(world_map);
    plotter->plot(world_map);
}

double Slam::calculate_cost(const PointCloud& cur_pc) const
{
    std::vector<uint32_t> nearest_indices;
    double cost = 0;

    for (size_t cur_idx=0; cur_idx<cur_pc.size(); cur_idx++) {
        double min = 9999999;
        uint32_t min_index = 0;
        for (size_t wld_idx=0; wld_idx<world_map.size(); wld_idx++){
            double dist = cur_pc.at(cur_idx).distance_to(world_map.at(wld_idx));
            if (dist < min){
                min = dist;
                min_index = wld_idx;
            }
        }
        nearest_indices.push_back(min_index);
        cost += min;
    }
    return cost;
}


/**
 * @brief ワールドマップと測定点群から現在位置を推定する
 */
void Slam::estimate_cur_pos(const PointCloud& cur_pc)
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