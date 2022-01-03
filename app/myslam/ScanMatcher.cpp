#include "ScanMatcher.hpp"
#include "Util.hpp"
#include <unistd.h>

void ScanMatcher::set_current_scan(const PointCloud *pc)
{
    this->cur_scan = pc;
}

void ScanMatcher::set_reference_scan(const PointCloud *pc)
{
    this->ref_scan = pc;
}

/**
 * @brief マッチングを終わらせる条件を決める。
 * @return 0 継続
 *         1- 終わる、数字は条件別
 */
uint32_t ScanMatcher::is_matching_done(
        const double ev,
        const double pre_ev,
        double *ev_history,
        uint32_t history_num,
        enum cost_type cost_type) const
{
    
    // 評価値がちょっとしか変化していない。
    if (std::abs(ev - pre_ev) < 1)
        return 1;

    // 過去N回分の評価値の移動平均が小さい
    double ave_ev = 0;
    for (uint32_t i=0; i<history_num; i++) {
        if (ev_history[i] != 0) {
            ave_ev += ev_history[i];
        }
    }

    ave_ev /= history_num;
    
    if (ev < 3000 && std::abs(ave_ev) < 100) {
        //printf("ave_ev=%f\n", ave_ev);
        return 2;
    }

    return false;
}

/**
 * @brief スキャンマッチング実行
 */
Pose2D ScanMatcher::do_scan_matching(const PointCloud *cur_scan, const PointCloud *ref_scan, double speed)
{
    this->set_current_scan(cur_scan);
    this->set_reference_scan(ref_scan);
    return do_scan_matching(speed);
}

/**
 * @brief スキャンマッチング実行
 */
Pose2D ScanMatcher::do_scan_matching(double speed) const
{
    std::vector<uint32_t> associate_list;
    PointCloud temp_scan = *this->cur_scan;
    enum cost_type cost_type = ScanMatcher::COST_SIMPLE;
    double pre_ev = 0;
    double ev = 0;
    double ev_history[10] = {0.0f};
    uint32_t history_num = sizeof(ev_history) / sizeof(double);
    Pose2D dev;
    Pose2D total_dev(0,0,0);

    if (this->is_debug_mode()) {
        this->debug_plotter->plot(this->ref_scan, this->cur_scan);
        sleep(speed);
    }

    int iter = 0;
    for (iter=0; iter<300; iter++) {

        // データを対応付ける。
        associate_list.clear();
        this->data_associate(&temp_scan, this->ref_scan, associate_list);

        // その対応付けでどのくらい遷移させれば最小コストになるか求める。
        dev = steepest_descent(&temp_scan, this->ref_scan, associate_list, cost_type);

        // 計算した分移動する。
        temp_scan.move(dev);



        // 移動後のコストを計算する
        pre_ev = ev;
        ev = this->cost_function(&temp_scan, ref_scan, associate_list, cost_type);

        // 評価値が小さすぎる場合はほぼ動いていないと判断して現在位置を動かさない。
        if (ev >= 2000) {
            total_dev.move_to(dev);
        }

        // コストの過去N回分を記録する
        for (int i = history_num - 1; i > 0; i--) {
            ev_history[i] = ev_history[i-1];
        }
        ev_history[0] = (ev - pre_ev);

        // 移動後の表示
        if (this->is_debug_mode()) {
            plot_for_debug(&temp_scan, this->ref_scan, associate_list);
            printf("[%d]dx=%f, dy=%f, dtheta=%f, cost_type=%d, ev=%f\n", iter, dev.x, dev.y, dev.direction, cost_type, ev);
            sleep(0.5 * speed);
        }

        // マッチングを終わらせるかチェック
        uint32_t done = is_matching_done(ev, pre_ev, ev_history, history_num, cost_type);
        if (done) {
            printf("done reason=%d, iter=%d\n", done, iter);
            break;
        }
    }

    //printf("done iter=%d ev=%f\n", iter, ev);

    //if (ev > 30000) {
    //    this->ref_scan->save_to_file("ref_scan.dat");
    //    this->cur_scan->save_to_file("cur_scan.dat");
    //}
    if (this->is_debug_mode())
        printf("Matching Done! dx=%f, dy=%f, dtheta=%f\n", total_dev.x, total_dev.y, total_dev.direction);

    return total_dev;
}

double ScanMatcher::differential(const PointCloud *scan,
                                 const PointCloud *ref_scan,
                                 const std::vector<uint32_t>& associate_list,
                                 double ev,
                                 double dd,
                                 double kk,
                                 enum cost_type cost_type) const
{
    double d_ev =
            (this->cost_function(scan, ref_scan, associate_list, cost_type) - ev) / dd;
    return -kk * d_ev;
}

/**
 * @brief 最急降下法によりscanとref_scanのズレを検出する
 * 
 * @param scan 現在スキャン
 * @param ref_scan 参照スキャン
 * @param associate_list 点の対応付けリスト
 * @param cost_type コストをどのタイプで計算するか
 * @return Pose2D 
 */
Pose2D ScanMatcher::steepest_descent(const PointCloud *scan,
                                     const PointCloud *ref_scan,
                                     const std::vector<uint32_t>& associate_list,
                                     const enum cost_type cost_type) const
{
    const double dd = 0.000001;
    const double da = COST_SIMPLE ? 5 : 2.5;
    const double kk = (cost_type == COST_SIMPLE) ? 0.00000001 : 0.00000001;
    PointCloud temp_scan = *scan;
    double ev = this->cost_function(&temp_scan, ref_scan, associate_list, cost_type);
    Pose2D total_mov(0,0,0);
    double min_ev = ev;
    double pre_ev = 0;
    Pose2D min_mov(0,0,0);
    int min_idx = 0;
    printf("[0]ev=%f, (0, 0, 0)\n", ev);

    for (int i=1; i<500 && std::abs(ev-pre_ev) > 0.01f; i++) {

        PointCloud dtheta_scan = temp_scan;
        PointCloud dx_scan = temp_scan;
        PointCloud dy_scan = temp_scan;

        dtheta_scan.rotate(dd);
        dx_scan.translate(da, 0);
        dy_scan.translate(0, da);

        double dtheta = differential(&dtheta_scan, ref_scan, associate_list, ev, dd, kk, cost_type);
        double dx = differential(&dx_scan, ref_scan, associate_list, ev, dd, kk, cost_type);
        double dy = differential(&dy_scan, ref_scan, associate_list, ev, dd, kk, cost_type);

        temp_scan.rotate(dtheta);
        temp_scan.translate(dx, 0);
        temp_scan.translate(0, dy);

        pre_ev = ev;
        ev = this->cost_function(&temp_scan, ref_scan, associate_list, cost_type);

        printf("[%d]ev=%f, (%f, %f, %f)\n", i, ev, dx, dy, dtheta);

        total_mov.x += dx;
        total_mov.y += dy;
        total_mov.direction += dtheta;

        if (ev < min_ev) {
            min_ev = ev;
            min_mov = total_mov;
            min_idx = i;
        }

    }
    printf("[min]%d(%f, %f, %f)\n", min_idx, min_mov.x, min_mov.y, min_mov.direction);
    return min_mov;
}

/**
 * @brief 対応点リストassociate_listに対して、
 * scanをどのくらい動かせばref_scanに対するコストが最小になるか求める
 */
Pose2D ScanMatcher::full_search(const PointCloud *scan,
                                const PointCloud *ref_scan,
                                const std::vector<uint32_t>& associate_list) const
{
    PointCloud temp_scan = *scan;
    double min_cost = 9999999999;
    double min_radian = 0;

    for (int i=0; i<1440; i++) {
        temp_scan.rotate(to_radian(0.25));
        double cost = this->cost_function(&temp_scan, ref_scan, associate_list, COST_SIMPLE);
        if (cost < min_cost) {
            min_cost = cost;
            min_radian = to_radian(i*0.25);
        }
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
    for (size_t i=0; i<cur_scan->size(); i++) {
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
    for (size_t i=0; i<pc->size(); i++) {
        const Point ref_point = pc->at(i);
        double dist = point.distance_to(ref_point);
        if (dist < min_dist && ref_point.type != PT_ISOLATE) {
            min_dist = dist;
            min_index = i;
        }
    }
    return min_index;
}

double ScanMatcher::simple_distance(const PointCloud *cur_scan,
                                    const PointCloud *ref_scan,
                                    const std::vector<uint32_t>& associate_list) const
{
    double dist_sum = 0;
    for (size_t i = 0; i < associate_list.size(); i++) {
        Point cur_point = cur_scan->at(i);
        Point ref_point = ref_scan->at(associate_list.at(i));
        double dist = cur_point.distance_to(ref_point);
        if (dist < 1000) {
            dist_sum += dist;
        }
    }
    return dist_sum;
}

double ScanMatcher::vertical_distance(const PointCloud *cur_scan,
                                      const PointCloud *ref_scan,
                                      const std::vector<uint32_t>& associate_list) const
{
    double dist_sum = 0;
    uint32_t nn = 0;
    for (size_t i = 0; i < associate_list.size(); i++) {
        Point cur_point = cur_scan->at(i);
        Point ref_point = ref_scan->at(associate_list.at(i));
        double dist = cur_point.vertical_distance_to(ref_point);
        
        if (dist < 100) {
            dist_sum += dist;
            nn++;
        }
    }
    return dist_sum / nn;
}

/**
 * @brief コスト関数
 */
double ScanMatcher::cost_function(const PointCloud *cur_scan,
                                  const PointCloud *ref_scan,
                                  const std::vector<uint32_t>& associate_list,
                                  const enum cost_type cost_type) const
{
    if (cost_type == COST_SIMPLE)
        return simple_distance(cur_scan, ref_scan, associate_list);
    else
        return vertical_distance(cur_scan, ref_scan, associate_list);
}

void ScanMatcher::plot_for_debug(const PointCloud *cur_scan,
                                 const PointCloud *ref_scan,
                                 const std::vector<uint32_t>& associate_list) const
{
    if (this->debug_plotter)
        this->debug_plotter->plot(cur_scan, ref_scan, associate_list);
}

