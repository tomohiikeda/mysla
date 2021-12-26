#include "Common.hpp"
#include "GlidMap.hpp"

/**
 * @brief コンストラクタ
 */
GlidMap::GlidMap(void)
{
    for (int x=0; x<this->map_size_x; x++) {
        for (int y=0; y<this->map_size_y; y++) {
            glid_map[x][y] = 0;
        }
    }
}

/**
 * @brief 格子地図に値をセットする。
 * @param world_pc 世界系座標の点群
 */
void GlidMap::set_points(const PointCloud *world_pc)
{
    for (int x=0; x<map_size_x; x++) {
        for (int y=0; y<map_size_y; y++) {
            if (this->glid_map[x][y])
                this->glid_map[x][y]--;
        }
    }

    for (size_t i=0; i<world_pc->size(); i++) {
        uint32_t x_index = (world_pc->at(i).x - map_min_x) / glid_per;
        uint32_t y_index = (world_pc->at(i).y - map_min_y) / glid_per;
        if (1 <= x_index && x_index < map_size_x-1) {
            if (1 <= y_index && y_index < map_size_y-1) {
                this->glid_map[x_index][y_index] = 3;
            }
        }
    }
}

void GlidMap::to_point_cloud(PointCloud *to_pc) const
{
    Point p(0, 0);
    for (int x=0; x<map_size_x; x++) {
        for (int y=0; y<map_size_y; y++) {
            if (this->glid_map[x][y]) {
                p.x = (double)x * glid_per + map_min_x + (glid_per / 2);
                p.y = (double)y * glid_per + map_min_y + (glid_per / 2);
                to_pc->add(p);
            }
        }
    }
    return;
}
