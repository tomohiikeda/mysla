#include <stdio.h>
#include <algorithm>
#include <fstream>
#include "GnuplotPlotter.hpp"

/**
 * @brief open gnuplot
 *
 * @return true succeeded
 * @return false failed
 */
bool GnuplotPlotter::open(void)
{
    fd = popen("gnuplot", "w");
    if (fd == NULL)
        return false;

    fprintf(fd, "set xr[%d:%d]\n", GridMap::map_min_x, GridMap::map_max_x);
    fprintf(fd, "set yr[%d:%d]\n", GridMap::map_min_y, GridMap::map_max_y);
    fprintf(fd, "set size square\n");
    fprintf(fd, "set term x11 size 1800,1800\n");
    fprintf(fd, "set zeroaxis\n");
    fflush(fd);
    return true;
}

/**
 * @brief close gnuplot
 *
 */
void GnuplotPlotter::close(void)
{
    if (fd == NULL)
        return;

    pclose(fd);
    fd = NULL;
    return;
}


void GnuplotPlotter::plot(const Pose2D& pose, const PointCloud& pc) const
{
    IPlotter::plot_info pose_info = {
        .label = "pose",
        .color = default_color,
        .line_width = default_line_width,
    };
    IPlotter::plot_info pc_info = {
        .label = "data",
        .color = default_color,
        .pt_size = default_pt_size,
        .pt_type = default_pt_type,
    };
    this->plot(pose, pose_info, pc, pc_info);
}

void GnuplotPlotter::plot(const Pose2D& pose, const struct IPlotter::plot_info& pose_info, const PointCloud& pc, const struct IPlotter::plot_info& pc_info) const
{
    if (fd == NULL)
        return;

    this->input_pose(pose, pose_info.label.c_str());
    this->input_points(pc, pc_info.label.c_str());
    fprintf(fd, "plot \
                \"$%s\" with lines linewidth %f linetype rgbcolor 0x%06x, \
                \"$%s\" with points pointtype %d pointsize %f\n",
                pose_info.label.c_str(),
                to_line_width(pose_info.line_width),
                to_color(pose_info.color),
                pc_info.label.c_str(),
                to_point_type(pc_info.pt_type),
                to_point_size(pc_info.pt_size));
    fflush(fd);
}

void GnuplotPlotter::plot(const PointCloud& pc) const
{
    IPlotter::plot_info pc_info = {
        .label = "data",
        .color = default_color,
        .pt_size = default_pt_size,
        .pt_type = default_pt_type,
    };
    this->plot(pc, pc_info);
}

void GnuplotPlotter::plot(const PointCloud& pc, const struct IPlotter::plot_info& pc_info) const
{
    if (fd == NULL)
        return;

    this->input_points(pc, pc_info.label.c_str());
    fprintf(fd, "plot \"$%s\" with points pointtype %d pointsize %f\n",
                pc_info.label.c_str(),
                to_point_type(pc_info.pt_type),
                to_point_size(pc_info.pt_size));
    fflush(fd);
}

void GnuplotPlotter::plot(const PointCloud& pc_0, const PointCloud& pc_1) const
{
    IPlotter::plot_info pc_info_0 = {
        .label = "data_0",
        .color = default_color,
        .pt_size = default_pt_size,
        .pt_type = default_pt_type,
    };
    IPlotter::plot_info pc_info_1 = {
        .label = "data_1",
        .color = default_color_2,
        .pt_size = default_pt_size,
        .pt_type = default_pt_type,
    };
    this->plot(pc_0, pc_info_0, pc_1, pc_info_1);
}

void GnuplotPlotter::plot(const PointCloud& pc_0, const struct IPlotter::plot_info& pc_info_0, const PointCloud& pc_1, const struct IPlotter::plot_info& pc_info_1) const
{
    if (fd == NULL)
        return;

    this->input_points(pc_0, pc_info_0.label.c_str());
    this->input_points(pc_1, pc_info_1.label.c_str());
    fprintf(fd, "plot \
                \"$%s\" with points pointtype %d pointsize %f, \
                \"$%s\" with points pointtype %d pointsize %f, \
                \n",
                pc_info_0.label.c_str(),
                to_point_type(pc_info_0.pt_type),
                to_point_size(pc_info_0.pt_size),
                pc_info_1.label.c_str(),
                to_point_type(pc_info_1.pt_type),
                to_point_size(pc_info_1.pt_size));
    fflush(fd);
}

void GnuplotPlotter::plot(const PointCloud& pc_0, const PointCloud& pc_1, const std::vector<uint32_t>& associate_list) const
{
    IPlotter::plot_info pc_info_0 = {
        .label = "data_0",
        .color = default_color,
        .pt_size = default_pt_size,
        .pt_type = default_pt_type,
    };
    IPlotter::plot_info pc_info_1 = {
        .label = "data_1",
        .color = default_color_2,
        .pt_size = default_pt_size,
        .pt_type = default_pt_type,
    };
    this->plot(pc_0, pc_info_0, pc_1, pc_info_1, associate_list);
}

void GnuplotPlotter::plot(const PointCloud& pc_0, const struct IPlotter::plot_info& pc_info_0, const PointCloud& pc_1, const struct IPlotter::plot_info& pc_info_1, const std::vector<uint32_t>& associate_list) const
{
    if (fd == NULL)
        return;

    this->input_points(pc_0, pc_info_0.label.c_str());
    this->input_points(pc_1, pc_info_1.label.c_str());
    this->input_associates(pc_0, pc_1, associate_list, "associate");
    fprintf(fd, "plot \
                \"$%s\" with points pointtype %d pointsize %f, \
                \"$%s\" with points pointtype %d pointsize %f, \
                \"$%s\" with linespoints pointtype 0\n",
                pc_info_0.label.c_str(),
                to_point_type(pc_info_0.pt_type),
                to_point_size(pc_info_0.pt_size),
                pc_info_1.label.c_str(),
                to_point_type(pc_info_1.pt_type),
                to_point_size(pc_info_1.pt_size),
                "associate");
    fflush(fd);
}

void GnuplotPlotter::plot(const Pose2D& pose, const GridMap& grid_map) const
{
    IPlotter::plot_info pose_info = {
        .label = "pose",
        .color = IPlotter::red,
        .line_width = default_line_width,
    };
    IPlotter::plot_info map_info = {
        .label = "map",
        .color = default_color,
        .pt_size = to_point_size(1.0f),
        .pt_type = IPlotter::round_fill,
    };
    this->plot(pose, pose_info, grid_map, map_info);
}

void GnuplotPlotter::plot(const Pose2D& pose, const struct IPlotter::plot_info& pose_info, const GridMap& grid_map, const struct IPlotter::plot_info& map_info) const
{
    if (fd == NULL)
        return;

    PointCloud pc;
    grid_map.to_point_cloud(&pc);
    this->plot(pose, pose_info, pc, map_info);
}

void GnuplotPlotter::input_pose(const Pose2D& pose, const char *data_var) const
{
    const double L = 100.0f;
    fprintf(fd, "$%s << EOD\n", data_var);
    fprintf(fd, "%f %f\n", pose.x - L * sin(pose.direction),
                           pose.y + L * cos(pose.direction));
    fprintf(fd, "%f %f\n", pose.x - (L / 3) * cos(pose.direction),
                           pose.y - (L / 3) * sin(pose.direction));
    fprintf(fd, "%f %f\n", pose.x + (L / 3) * cos(pose.direction),
                           pose.y + (L / 3) * sin(pose.direction));
    fprintf(fd, "%f %f\n", pose.x - L * sin(pose.direction),
                           pose.y + L * cos(pose.direction));
    fprintf(fd, "EOD\n");
}

void GnuplotPlotter::input_points(const PointCloud& pc, const char *data_var) const
{
    fprintf(fd, "$%s << EOD\n", data_var);
    for (size_t i = 0; i< pc.size(); i++) {
        fprintf(fd, "%f %f\n", pc.at(i).x, pc.at(i).y);
    }
    fprintf(fd, "EOD\n");
}

void GnuplotPlotter::input_associates(const PointCloud& cur_pc, const PointCloud& ref_pc,
            const std::vector<uint32_t>& associate_list, const char *data_var) const
{
    fprintf(fd, "$%s << EOD\n", data_var);

    for (size_t i = 0; i < associate_list.size(); i++) {

        Point cur_point = cur_pc.at(i);
        fprintf(fd, "%f %f\n", cur_point.x, cur_point.y);

        Point ref_point;
        if (ref_pc.size() > associate_list.at(i)) {
            ref_point = ref_pc.at(associate_list.at(i));
            fprintf(fd, "%f %f\n", ref_point.x, ref_point.y);
        }

        fprintf(fd, "\n");
    }

    fprintf(fd, "EOD\n");
}

void GnuplotPlotter::input_normal(const PointCloud& pc, const char *data_var) const
{
    fprintf(fd, "$%s << EOD\n", data_var);
    for (size_t i = 0; i < pc.size(); i++) {
        Point pt = pc.at(i);
        if (pt.normal.x || pt.normal.y) {
            fprintf(fd, "%f %f\n", pt.x, pt.y);
            fprintf(fd, "%f %f\n", pt.x + pt.normal.x * 100, pt.y + pt.normal.y * 100);
            fprintf(fd, "\n");
        }
    }
    fprintf(fd, "EOD\n");
}

double GnuplotPlotter::to_point_size(double pt_size) const
{
    return pt_size;
}

double GnuplotPlotter::to_line_width(double line_width) const
{
    return line_width;
}

uint32_t GnuplotPlotter::to_point_type(point_type pt_type) const
{
    return (uint32_t)pt_type;
}

uint32_t GnuplotPlotter::to_color(color color) const
{
    switch(color){
    case red: return 0xff0000;
    case green: return 0x00ff00;
    case blue: return 0x0000ff;
    case cyan: return 0x00ffff;
    case magenta: return 0xff00ff;
    case yellow: return 0xffff00;
    case black: return 0xffffff;
    case purple: return 0xa020f0;
    default: return 0xffffff;
    }
}
