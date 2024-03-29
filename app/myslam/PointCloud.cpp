#include "PointCloud.hpp"
#include "Util.hpp"

void PointCloud::add(Point point)
{
    points.push_back(point);
}

void PointCloud::add(const PointCloud *pc)
{
    for (uint32_t i=0; i<pc->size(); i++){
        points.push_back(pc->at(i));
    }
}

const Point& PointCloud::at(int x) const
{
    return points.at(x);
}

size_t PointCloud::size(void) const
{
    return points.size();
}

void PointCloud::clear(void)
{
    points.clear();
}

void PointCloud::copy_to(PointCloud& to) const
{
    to.clear();
    for (size_t i=0; i<points.size(); i++)
        to.add(points.at(i));
}

void PointCloud::translate(const double x, const double y)
{
    for (size_t i=0; i<points.size(); i++) {
        points.at(i).x += x;
        points.at(i).y += y;
    }
}

void PointCloud::thin_out(uint32_t interval)
{
    std::vector<Point> tmp_points;
    for (size_t i=0; i<points.size()-1; i++) {
        if ((i % interval) == 0) {
            tmp_points.push_back(points.at(i));
        }
    }
    this->points.clear();
    std::copy(tmp_points.begin(), tmp_points.end(), back_inserter(points));
}

void PointCloud::rotate(const double radian)
{
    for (size_t i=0; i<points.size(); i++) {
        double x = points.at(i).x;
        double y = points.at(i).y;
        points.at(i).x  = x * std::cos(radian) - y * std::sin(radian);
        points.at(i).y  = x * std::sin(radian) + y * std::cos(radian);
    }
}

void PointCloud::move(const Pose2D& movement)
{
    this->rotate(movement.direction);
    this->translate(movement.x, movement.y);
}

void PointCloud::move(const Movement2D& movement)
{
    for (size_t i=0; i<points.size(); i++) {
        Point& p = points.at(i);
        p.move(movement);
    }
}

void PointCloud::save_to_file(const char *filename) const
{
    std::ofstream ofs(filename);
    for (size_t i=0; i<points.size(); i++){
        ofs << points.at(i).x << " " << points.at(i).y << std::endl;
    }
    ofs.close();
}

void PointCloud::load_from_file(const char *filename)
{
    std::ifstream ifs(filename);
    if (!ifs)
        return;

    std::string line;
    while (getline(ifs, line)) {
        std::stringstream ss;
        std::string s, x, y;
        ss << line;
        getline(ss, x, ' ');
        getline(ss, y, ' ');
        Point p(stod(x), stod(y));
        this->add(p);
    }
    ifs.close();
}

void PointCloud::analyse_points(void)
{
    for (size_t i=0; i<points.size(); i++) {

        Point& pt = points.at(i);
        Vector2D nl, nr;

        bool flag_nl = this->calculate_normal(nl, i, pt, -1);
        bool flag_nr = this->calculate_normal(nr, i, pt, 1);
        nr.x = -nr.x;
        nr.y = -nr.y;

        if (flag_nl & flag_nr) {
            if (fabs(nl.x * nr.x + nl.y * nr.y) >= cos(to_radian(45))) {
                pt.type = PT_LINE; // 2つの法線が並行に近い
                double dx = nl.x + nr.x;
                double dy = nl.y + nr.y;
                double L = sqrt(dx * dx + dy * dy);
                pt.normal.x = dx / L;
                pt.normal.y = dy / L;
            } else {
                pt.type = PT_CORNER; // 2つの法線が平行でない
            }
        } else if (flag_nl) {
            pt.type = PT_LINE;
            pt.normal = nl;
        } else if (flag_nr) {
            pt.type = PT_LINE;
            pt.normal = nr;
        } else {
            pt.type = PT_ISOLATE;
        }
    }
}

void PointCloud::trim(double min_x, double max_x, double min_y, double max_y)
{
    std::vector<Point>::iterator it = this->points.begin();

    while (it != this->points.end()) {
        Point p = *it;
        if (p.x < min_x || max_x <= p.x || p.y < min_y || max_y <= p.y)
            it = this->points.erase(it);
        else
            it++;
    }

    this->points.shrink_to_fit();
}

bool PointCloud::calculate_normal(Vector2D& normal, int idx, const Point& pt, int dir)
{
    const double FPDMIN = 0.06;
    const double FPDMAX = 100.0;

    for (size_t i=idx; 0<=i && i<this->points.size(); i+=dir) {
        const Point& lp = this->points.at(i);
        double dx = lp.x - pt.x;
        double dy = lp.y - pt.y;
        double d = sqrt(dx*dx + dy*dy);
        if (FPDMIN <= d && d <= FPDMAX) {
            normal.x = dy/d;
            normal.y = -dx/d;
            return true;
        } else if (FPDMAX < d) {
            return false;
        }
    }
    return false;
}

void PointCloud::print(void) const
{
    for (size_t i=0; i<points.size(); i++) {
        Point pt = points.at(i);
        printf("[%d]x=%f, y=%f, normal={%f,%f}, type=%d\n", i, pt.x, pt.y, pt.normal.x, pt.normal.y, pt.type);
    }
}

