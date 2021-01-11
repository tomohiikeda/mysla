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
    Pose2D total_dev(0,0,0);

    //this->debug_plotter->plot(&temp_scan, this->ref_scan);
    //sleep(3);

    for (int iter=0; iter<100; iter++) {
        
        // データを対応付ける。
        associate_list.clear();
        this->data_associate(&temp_scan, this->ref_scan, associate_list);

        // その対応付けでどのくらい遷移させれば最小コストになるか求める。
        Pose2D dev = steepest_descent(&temp_scan, this->ref_scan, associate_list);
        //printf("[%d]dx=%f, dy=%f, dtheta=%f\n", iter, dev.x, dev.y, dev.direction);
        
        if (std::abs(dev.direction) < 0.0005)
            break;
        
        // 計算した分移動する。
        temp_scan.move(dev);
        
        total_dev.move_to(dev);

        //this->debug_plotter->plot(&temp_scan, this->ref_scan, associate_list);
        //sleep(0.3);
    }

    return total_dev;
}

/**
 * @brief 対応点リストassociate_listに対して、
 * scanをどのくらい動かせばref_scanに対するコストが最小になるか求める
 */
Pose2D ScanMatcher::steepest_descent(const PointCloud *scan,
                                     const PointCloud *ref_scan,
                                     const std::vector<uint32_t>& associate_list) const
{
    const double dd = 0.001;
    const double da = 10;
    const double kk = 0.0000001;
    PointCloud temp_scan = *scan;
    double ev = this->cost_function(&temp_scan, ref_scan, associate_list);
    double total_dtheta = 0;
    double total_dx = 0;
    double total_dy = 0;

    for (int i=0; i<30; i++) {
        
        PointCloud dtheta_scan = temp_scan;
        dtheta_scan.rotate(dd);
        double dtheta_ev =
             (this->cost_function(&dtheta_scan, ref_scan, associate_list) - ev) / dd;
        double dtheta = -kk * dtheta_ev;
        temp_scan.rotate(dtheta);
        total_dtheta += dtheta;

        PointCloud dx_scan = temp_scan;
        dx_scan.translate(da, 0);
        double dx_ev =
             (this->cost_function(&dx_scan, ref_scan, associate_list) - ev) / da;
        double dx = -kk * dx_ev;
        temp_scan.translate(dx, 0);
        total_dx += dx;

        PointCloud dy_scan = temp_scan;
        dy_scan.translate(0, da);
        double dy_ev =
             (this->cost_function(&dy_scan, ref_scan, associate_list) - ev) / da;
        double dy = -kk * dy_ev;
        temp_scan.translate(0, dy);
        total_dy += dy;

        ev = this->cost_function(&temp_scan, ref_scan, associate_list);
        
        //this->plot_for_debug(&temp_scan, ref_scan, associate_list);
        //sleep(0.5);
    }
    Pose2D dev(total_dx, total_dy, total_dtheta);
    return dev;


#if 0
    PointCloud temp_scan = *scan;
    double min_cost = 9999999999;
    double max_cost = 0;
    double min_radian = 0;

    for (int i=0; i<1440; i++) {
        temp_scan.rotate(to_radian(0.25));
        double cost = this->cost_function(&temp_scan, ref_scan, associate_list);
        //plot_for_debug(&temp_scan, this->ref_scan, associate_list);
        if (cost < min_cost) {
            min_cost = cost;
            min_radian = to_radian(i*0.25);
        }
    }
    Pose2D dev(0, 0, min_radian);
    return dev;

#endif

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
    if (this->debug_plotter)
        this->debug_plotter->plot(cur_scan, ref_scan, associate_list);
}

