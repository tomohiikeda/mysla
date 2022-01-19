#include "Common.hpp"
#include <iostream>
#include <unistd.h>
#include <time.h>
#include <sys/time.h>
#include "Slam.hpp"
#include "PoseEstimator.hpp"
#include "GnuplotPlotter.hpp"
#include "GridMap.hpp"

/**
 * @brief SLAM初期化
 */
bool Slam::init(void)
{
    if(retriever.init() == false)
        return false;

    if(plotter.open() == false)
        return false;

    return true;
}


/**
 * @brief SLAM開始
 */
bool Slam::start(void)
{
    if(retriever.start() == false)
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
    retriever.stop();
    plotter.close();
}


/**
 * @brief SLAMの実行スレッド
 */
void Slam::process_loop(void)
{
    ScanMatcher scan_matcher(this->plotter, this->debug);
    PoseEstimator pose_estimator(scan_matcher);
    uint32_t loop_num = 0;
    Pose2D cur_pose(0, 0, 0);
    GridMap *world_grid_map = new GridMap();

    this->running = true;

    this->load_from_file("slam", cur_pose, *world_grid_map);

    // ずっとループ
    while (this->running == true) {

        struct timeval timeval;
        gettimeofday(&timeval, NULL);
        printf("//-------------------------------------------------------\n");
        printf("//  Index = %d (%fmm, %fmm, %fdeg)\n", loop_num, cur_pose.x, cur_pose.y, to_degree(cur_pose.direction));
        printf("//-------------------------------------------------------\n");

        // 最新のSlamDataを取得する。
        SlamData slam_data;
        if (retriever.retrieve(slam_data) == false)
            break;

        //slam_data.pc()->trim(-1500, 1500, -1500, 1500);

        // 最新SlamDataとワールドマップから現在位置を推定する。
        cur_pose = pose_estimator.estimate_position(cur_pose, slam_data, *world_grid_map);

        printf("before\n");
        slam_data.pc()->print();

        // ワールドマップを更新する。
        slam_data.pc()->move(cur_pose);

        printf("after\n");
        slam_data.pc()->print();

        world_grid_map->set_points(slam_data.pc());

        // 現在位置とワールドマップを表示する。
        //if (!this->debug)
            plotter.plot(cur_pose, *world_grid_map);

        loop_num++;

        //sleep(1);
        //wait_for_key();
    }

    //this->save_to_file("slam", cur_pose, *world_grid_map);

    delete world_grid_map;
    return;
}

void Slam::load_from_file(const std::string filename, Pose2D& pose, GridMap& map) const
{
    pose.load_from_file(filename + "_pose.dat");
    map.load_from_file(filename + "_gridmap.dat");
}

void Slam::save_to_file(const std::string filename, const Pose2D& pose, const GridMap& map) const
{
    pose.save_to_file(filename + "_pose.dat");
    map.save_to_file(filename + "_gridmap.dat");
}


/**
 * @brief スレッドエントリ関数
 */
void *Slam::thread_entry(void *arg)
{
    ((Slam*)arg)->process_loop();
    return NULL;
}
