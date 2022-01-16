#include "GridMap.hpp"

void Grid::set(const Point p)
{
    uint32_t num = this->point_num;

    this->represent.x
    = this->represent.x * ((double)num / (num + 1)) + p.x * (1.0f / (num + 1));

    this->represent.y
    = this->represent.y * ((double)num / (num + 1)) + p.y * (1.0f / (num + 1));

    this->point_num++;
}

Point Grid::get(void) const
{
    return this->represent;
}

bool Grid::is_valid(void) const
{
    return (this->point_num) ? true : false;
}

int32_t GridMap::to_index_x(const double world_x) const
{
    if (world_x < GridMap::map_min_x || GridMap::map_max_x <= world_x)
        return -1;
    else
        return (world_x - map_min_x) / Grid::width;
}

int32_t GridMap::to_index_y(const double world_y) const
{
    if (world_y < GridMap::map_min_y || GridMap::map_max_y <= world_y)
        return -1;
    else
        return (world_y - map_min_y) / Grid::height;
}

/**
 * @brief 格子地図に値をセットする。
 * @param world_pc 世界系座標の点群
 */
void GridMap::set_points(const PointCloud *world_pc)
{
    for (size_t i=0; i<world_pc->size(); i++) {

        //if (world_pc->at(i).type == PT_ISOLATE)
        //    continue;

        Point p = world_pc->at(i);
        int32_t index_x = to_index_x(p.x);
        int32_t index_y = to_index_x(p.y);

        if (index_x == -1 || index_y == -1)
            return;

        this->grid_map[index_x][index_y].set(p);
    }
}

void GridMap::to_point_cloud(PointCloud *to_pc) const
{
    for (uint32_t x=0; x<max_grid_index_x; x++) {
        for (uint32_t y=0; y<max_grid_index_y; y++) {
            if (this->grid_map[x][y].is_valid()) {
                Point p = this->grid_map[x][y].get();
                to_pc->add(p);
            }
        }
    }
}
