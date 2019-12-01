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
    PointCloud cur_pc;

    if (sensor->get_point_cloud(pre_pc) == false)
        return;

    pre_pc.copy_to(world_map);  // 初回

    while(running){

        if (sensor->get_point_cloud(cur_pc) == false) {
            running = false;
            return;
        }

        update_world_map(cur_pc);
        estimate_cur_pose(cur_pc);

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
    Pose2D deviation = calc_deviation_from_world(cur_pc);

    PointCloud move_pc = cur_pc;
    move_pc.rotate(deviation.direction);
    move_pc.move(deviation.x, deviation.y);

    grow_world_map(move_pc);

    plotter->plot(world_map);
}

/**
 * @brief 現在点群とワールドマップがどのくらいズレているか計算する
 * @param pc 現在点群
 * @return ズレ量
 */
Pose2D Slam::calc_deviation_from_world(const PointCloud& pc) const
{
    PointCloud rot_pc = pc;
    double min = 9999999999;
    double min_degree = 0;
    const double degree_inc = 0.5f;

    for (double degree=0; degree<360; degree+=degree_inc) {
        rot_pc.rotate(to_radian(degree_inc));
        double cost = calculate_cost(rot_pc);
        if (cost < min) {
            min = cost;
            min_degree = degree;
        }
    }

    std::cout << min_degree << std::endl;
    Pose2D pose(0, 0, to_radian(min_degree));
    return pose;
}

/**
 * @brief ワールドマップに現在点群を足して成長させる。
 * @param cur_pc 成長させる点群
 */
void Slam::grow_world_map(const PointCloud& cur_pc)
{
    world_map.add(cur_pc);
    world_map.optimize();
    
    return;
}


/**
 * @brief 入力点群がワールドマップに対してどのくらいズレているかを表示する
 * @param pc 入力点群
 * @return ワールドマップに対する入力点群のズレ(コスト)
 */
double Slam::calculate_cost(const PointCloud& pc) const
{
    std::vector<uint32_t> nearest_indices;
    double cost = 0;

    for (size_t cur_idx=0; cur_idx<pc.size(); cur_idx++) {
        double min = 9999999;
        uint32_t min_index = 0;
        for (size_t wld_idx=0; wld_idx<world_map.size(); wld_idx++){
            double dist = pc.at(cur_idx).distance_to(world_map.at(wld_idx));
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
void Slam::estimate_cur_pose(const PointCloud& cur_pc)
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