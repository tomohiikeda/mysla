#include "GridMap.hpp"

void Grid::init(const double x, const double y, const uint32_t num)
{
    this->represent.x = x;
    this->represent.y = y;
    this->point_num = num;
}

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

bool Grid::is_tentative(void) const
{
    return (this->point_num == 1) ? true : false;
}

std::string Grid::to_string(void) const
{
    std::string str = std::to_string(represent.x) + " " +
                      std::to_string(represent.y) + " " +
                      std::to_string(point_num);
    return str;
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
        Point p = world_pc->at(i);
        int32_t index_x = to_index_x(p.x);
        int32_t index_y = to_index_x(p.y);
        if (index_x == -1 || index_y == -1)
            continue;
        this->grid_map[index_x][index_y].set(p);
    }

    // 2回連続でセットされなかったGridは初期化する。
    if (this->pre_pc.size()) {
        for (size_t i=0; i<pre_pc.size(); i++) {
            const Point& p = pre_pc.at(i);
            int32_t index_x = to_index_x(p.x);
            int32_t index_y = to_index_y(p.y);
            if (index_x == -1 || index_y == -1)
                continue;
            if (this->grid_map[index_x][index_y].is_tentative())
                this->grid_map[index_x][index_y].init(0, 0, 0);
        }
    }

    world_pc->copy_to(this->pre_pc);
}

void GridMap::to_point_cloud(PointCloud *to_pc) const
{
    this->to_point_cloud(to_pc, map_min_x, map_max_x-1, map_min_y, map_max_y-1);
}

void GridMap::to_point_cloud(PointCloud *to_pc, double min_x, double max_x, double min_y, double max_y) const
{
    int32_t min_index_x = (to_index_x(min_x) != -1) ? to_index_x(min_x) : 0;
    int32_t max_index_x = (to_index_x(max_x) != -1) ? to_index_x(max_x) : max_grid_index_x;
    int32_t min_index_y = (to_index_y(min_y) != -1) ? to_index_y(min_y) : 0;
    int32_t max_index_y = (to_index_y(max_y) != -1) ? to_index_y(max_y) : max_grid_index_y;

    for (int32_t x=min_index_x; x<max_index_x; x++) {
        for (int32_t y=min_index_y; y<max_index_y; y++) {
            if (this->grid_map[x][y].is_valid() && !this->grid_map[x][y].is_tentative()) {
                Point p = this->grid_map[x][y].get();
                to_pc->add(p);
            }
        }
    }
}

void GridMap::save_to_file(const std::string filename) const
{
    std::ofstream ofs(filename);
    if (!ofs)
        return;

    for (uint32_t y=0; y<max_grid_index_y; y++) {
        for (uint32_t x=0; x<max_grid_index_x; x++) {
            Grid g = grid_map[x][y];
            ofs << g.to_string() << std::endl;
        }
    }
    ofs.close();
}

void GridMap::load_from_file(const std::string filename)
{
    std::ifstream ifs(filename);
    if (!ifs)
        return;

    for (uint32_t y=0; y<max_grid_index_y; y++) {
        for (uint32_t x=0; x<max_grid_index_x; x++) {
            std::string line;
            std::stringstream ss;
            std::string str_x, str_y, str_n;
            getline(ifs, line);
            ss << line;
            getline(ss, str_x, ' ');
            getline(ss, str_y, ' ');
            getline(ss, str_n, ' ');
            grid_map[x][y].init(stod(str_x), stod(str_y), (uint32_t)stoi(str_n));
        }
    }

    ifs.close();
}
