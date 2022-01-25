#include "Pose2D.hpp"

Pose2D::Pose2D(void)
{
    Pose2D(0, 0, 0);
}

Pose2D::Pose2D(double x, double y, double dir)
{
    this->x = x;
    this->y = y;
    this->direction = dir;
}

void Pose2D::print(void) const
{
    printf("%lf, %lf, %lf\n", x, y, direction);
}

void Pose2D::print(uint32_t num) const
{
    printf("[%05d]%lf, %lf, %lf(%lf degree)\n", num, x, y, direction, to_degree(direction));
}

void Pose2D::move(const Movement2D& movement)
{
    Eigen::Matrix<double, 3, 1> pose;
    pose << this->x, this->y, 1;
    pose = movement.move_matrix * pose;
    this->x = pose(0, 0);
    this->y = pose(1, 0);
    this->direction += movement.rotate;
}

void Pose2D::set(double x, double y, double dir)
{
    this->direction = dir;
    this->x = x;
    this->y = y;
}

void Pose2D::save_to_file(const std::string filename) const
{
    std::ofstream ofs(filename);
    ofs << x << " " << y << " " << direction << std::endl;
    ofs.close();
}

void Pose2D::load_from_file(const std::string filename)
{
    std::ifstream ifs(filename);
    if (!ifs)
        return;
    std::string line;
    std::stringstream ss;
    std::string str_x, str_y, str_d;
    getline(ifs, line);
    ss << line;
    getline(ss, str_x, ' ');
    getline(ss, str_y, ' ');
    getline(ss, str_d, ' ');
    this->x = std::stod(str_x);
    this->y = std::stod(str_y);
    this->direction = std::stod(str_d);
    ifs.close();
}
