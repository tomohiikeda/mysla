#include "SlamData.hpp"

SlamData::SlamData(void)
{
    return;
}

PointCloud *SlamData::pc(void)
{
    return &(this->point_cloud);
}

odometory_t *SlamData::odometory(void)
{
    return &(this->odom);
}

void SlamData::save_to_file(const char *filename) const
{
    std::ofstream ofs(filename);
    ofs << this->odom.left << " " << this->odom.right << std::endl;
    for (size_t i=0; i<this->point_cloud.size(); i++){
        ofs << this->point_cloud.at(i).x << " " << this->point_cloud.at(i).y << std::endl;
    }
    ofs.close();
}

bool SlamData::load_from_file(const char *filename)
{
    std::stringstream ss;
    std::string x, y, line;
    std::ifstream ifs(filename);
    
    if (!ifs || ifs.fail())
        return false;

    // 1行目はodometory
    if (!getline(ifs, line))
        return false;

    ss << line;
    getline(ss, x, ' ');
    getline(ss, y, ' ');
    this->odom.left = stoi(x);
    this->odom.right = stoi(y);

    // 2行目以降はPoint
    while (getline(ifs, line)) {
        std::stringstream ss2;
        ss2 << line;
        getline(ss2, x, ' ');
        getline(ss2, y, ' ');
        Point p(stod(x), stod(y));
        this->point_cloud.add(p);
    }
    ifs.close();
    return true;
}

void SlamData::print(void) const
{
    printf("odometory %d %d\n", this->odom.left, this->odom.right);
    printf("point size = %d\n", this->point_cloud.size());
    this->point_cloud.print();
}
