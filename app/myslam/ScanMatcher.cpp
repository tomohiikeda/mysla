#include "ScanMatcher.hpp"
#include "Common.h"
#include <unistd.h>

void ScanMatcher::set_debug_plotter(const IPlotter *plotter)
{
    this->debug_plotter = plotter;
}

void ScanMatcher::set_current_scan(const PointCloud *pc)
{
    this->cur_scan = pc;
    printf("Current Scan size = %d\n", pc->size());
}

void ScanMatcher::set_reference_scan(const PointCloud *pc)
{
    this->ref_scan = pc;
    printf("Reference Scan size = %d\n", pc->size());
}

/**
 * @brief スキャンマッチング実行
 */
Pose2D ScanMatcher::do_scan_matching(void) const
{
    PointCloud temp_scan = *this->cur_scan;
    std::vector<uint32_t> associate_list;

    for (int iter=0; iter<100; iter++) {
        
        // データを対応付ける。
        associate_list.clear();
        this->data_associate(&temp_scan, this->ref_scan, associate_list);

        // その対応付けでどのくらい遷移させれば最小コストになるか求める。
        Pose2D dev = minimize_cost_pose(&temp_scan, this->ref_scan, associate_list);
        printf("[%]min_radian=%f\n", iter, dev.direction);
        
        if (dev.direction == 0)
            break;
        // 計算した分移動する。
        temp_scan.rotate(dev.direction);
        temp_scan.move(dev.x, dev.y);
        this->debug_plotter->plot(&temp_scan, this->ref_scan, associate_list);
        sleep(3);
    }


    Pose2D pose;
    return pose;
}

/**
 * @brief scanをどのくらい動かせばref_scanに対するコストが最小になるか求める
 */
Pose2D ScanMatcher::minimize_cost_pose(const PointCloud *scan,
                                       const PointCloud *ref_scan,
                                       const std::vector<uint32_t>& associate_list) const
{
    PointCloud temp_scan = *scan;
    double min_cost = 9999999999;
    double max_cost = 0;
    double min_radian = 0;

    for (int i=0; i<1440; i++) {
        temp_scan.rotate(to_radian(0.25));
        double cost = this->cost_function(&temp_scan, ref_scan, associate_list);
        //plot_for_debug(&temp_scan, this->ref_scan, associate_list);
        //sleep(0.5);
        if (cost < min_cost) {
            min_cost = cost;
            min_radian = to_radian(i*0.25);
        }
        //wait_for_key();
    }

    Pose2D dev(0, 0, min_radian);
    return dev;
}

/**
 * @brief 参照スキャンに対する現在スキャンの点の対応付け
 */
void ScanMatcher::data_associate(const PointCloud *cur_scan,
                                 const PointCloud *ref_scan,
                                 std::vector<uint32_t>& associate_list) const
{
    for (int i=0; i<cur_scan->size(); i++) {
        Point cur_point = cur_scan->at(i);
        uint32_t nearest_index = find_nearest_index(cur_point, this->ref_scan);
        associate_list.push_back(nearest_index);
    }
}


/**
 * @brief pcの中からpointに一番近い点を返す関数
 */
uint32_t ScanMatcher::find_nearest_index(const Point point, const PointCloud *pc) const
{
    double min_dist = 999999999;
    uint32_t min_index = 0;
    for (int i=0; i<pc->size(); i++) {
        const Point ref_point = pc->at(i);
        double dist = point.distance_to(ref_point);
        if (dist < min_dist) {
            min_dist = dist;
            min_index = i;
        }
    }
    return min_index;
}

/**
 * @brief コスト関数
 */
double ScanMatcher::cost_function(const PointCloud *cur_scan,
                                  const PointCloud *ref_scan,
                                  const std::vector<uint32_t>& associate_list) const
{
    double dist_sum = 0;
    for (size_t i = 0; i < associate_list.size(); i++) {
        Point cur_point = cur_scan->at(i);
        Point ref_point = ref_scan->at(associate_list.at(i));
        double dist = cur_point.distance_to(ref_point);
        dist_sum += dist;
    }
    return dist_sum;
}

void ScanMatcher::plot_for_debug(const PointCloud *cur_scan,
                                 const PointCloud *ref_scan,
                                 const std::vector<uint32_t>& associate_list) const
{
    this->debug_plotter->plot(cur_scan, ref_scan, associate_list);
}

