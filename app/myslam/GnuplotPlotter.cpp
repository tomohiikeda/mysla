#include <stdio.h>
#include <algorithm>
#include <fstream>
#include "GnuplotPlotter.hpp"

bool GnuplotPlotter::open(void)
{
    fd = popen("gnuplot", "w");
    if (fd == NULL)
        return false;

    fprintf(fd, "set xr[-3000:3000]\n");
    fprintf(fd, "set yr[-3000:3000]\n");
    //fprintf(fd, "set xr[-1:1]\n");
    //fprintf(fd, "set yr[-1:1]\n");
    fprintf(fd, "set size square\n");
    fflush(fd);
    return true;
}

void GnuplotPlotter::close(void)
{
    if (fd == NULL)
        return;

    pclose(fd);
    fd = NULL;
    return;
}

/**
 * @brief プロット関数。自機をプロットする。
 * @param pose 自機の現在Pose
 */
void GnuplotPlotter::plot(const Pose2D pose) const
{
    if (fd == NULL)
        return;
    
    fprintf(fd, "$pose << EOD\n");
    fprintf(fd, "%f %f\n", pose.x, pose.y);
    fprintf(fd, "EOD\n");
    fprintf(fd, "plot \"$pose\" with points pointtype 1 pointsize 1\n");
    fflush(fd);
}

void GnuplotPlotter::plot(const Pose2D pose, const PointCloud *pc) const
{
    if (fd == NULL)
        return;

    const char *pose_var = "pose";
    const char *data_var = "data";
    this->input_pose(pose, pose_var);
    this->input_points(pc, data_var);
    fprintf(fd, "plot \
                \"$%s\" with lines, \
                \"$%s\" with points pointtype 7 pointsize 0.2\n", pose_var, data_var);
}

void GnuplotPlotter::plot(const PointCloud *pc) const
{
    if (fd == NULL)
        return;

    const char *data_var = "data";
    this->input_points(pc, data_var);

    fprintf(fd, "plot \"$%s\" with points pointtype 7 pointsize 0.2\n", data_var);
    return;
}

void GnuplotPlotter::plot(const PointCloud *pc_0, const PointCloud *pc_1) const
{
    if (fd == NULL)
        return;

    const char *plotfile_0 = "data_0";
    const char *plotfile_1 = "data_1";
    const char *normalfile_0 = "normal";
    this->input_points(pc_0, plotfile_0);
    this->input_points(pc_1, plotfile_1);
    this->input_normal(pc_1, normalfile_0);

    /*
    fprintf(fd, "plot \
                \"$%s\" with points pointtype 7 pointsize 0.2, \
                \"$%s\" with points pointtype 7 pointsize 0.2, \
                \"$%s\" with linespoints pointtype 0 \
                \n",
            plotfile_0, plotfile_1, normalfile_0);
    */
    fprintf(fd, "plot \
                \"$%s\" with points pointtype 7 pointsize 0.2, \
                \"$%s\" with points pointtype 7 pointsize 0.2, \
                \n",
            plotfile_0, plotfile_1);
    return;
}

void GnuplotPlotter::plot(const PointCloud *pc_0,
                          const PointCloud *pc_1,
                          const std::vector<uint32_t>& associate_list) const
{
    if (fd == NULL)
        return;

    const char *plotfile_0 = "data_0";
    const char *plotfile_1 = "data_1";
    const char *plotfile_2 = "associates";
    this->input_points(pc_0, plotfile_0);
    this->input_points(pc_1, plotfile_1);
    this->input_associates(pc_0, pc_1, associate_list, plotfile_2);

    fprintf(fd, "plot \
                \"$%s\" with points pointtype 7 pointsize 0.2, \
                \"$%s\" with points pointtype 7 pointsize 0.2, \
                \"$%s\" with linespoints pointtype 0\n",
            plotfile_0, plotfile_1, plotfile_2);
    return;
}

void GnuplotPlotter::plot(const GlidMap& glid_map) const
{
    if (fd == NULL)
        return;

    PointCloud pc;
    glid_map.to_point_cloud(&pc);
    this->plot(&pc);

    return;
}

void GnuplotPlotter::plot(const Pose2D pose, const GlidMap& glid_map) const
{
    if (fd == NULL)
        return;

    PointCloud pc;
    glid_map.to_point_cloud(&pc);
    this->plot(pose, &pc);

    return;
}

void GnuplotPlotter::input_pose(const Pose2D pose, const char *data_var) const
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

void GnuplotPlotter::input_points(const PointCloud *pc, const char *data_var) const
{
    fprintf(fd, "$%s << EOD\n", data_var);
    for (size_t i = 0; i< pc->size(); i++) {
        fprintf(fd, "%f %f\n", pc->at(i).x, pc->at(i).y);
    }
    fprintf(fd, "EOD\n");
}

void GnuplotPlotter::input_associates(const PointCloud *cur_pc, const PointCloud *ref_pc,
            const std::vector<uint32_t>& associate_list, const char *data_var) const
{
    fprintf(fd, "$%s << EOD\n", data_var);
    for (size_t i = 0; i < associate_list.size(); i++) {
        Point cur_point = cur_pc->at(i);
        Point ref_point = ref_pc->at(associate_list.at(i));
        fprintf(fd, "%f %f\n", cur_point.x, cur_point.y);
        fprintf(fd, "%f %f\n", ref_point.x, ref_point.y);
        fprintf(fd, "\n");
    }
    fprintf(fd, "EOD\n");
}

void GnuplotPlotter::input_normal(const PointCloud *pc, const char *data_var) const
{
    fprintf(fd, "$%s << EOD\n", data_var);
    for (size_t i = 0; i < pc->size(); i++) {
        Point pt = pc->at(i);
        if (pt.normal.x || pt.normal.y) {
            fprintf(fd, "%f %f\n", pt.x, pt.y);
            fprintf(fd, "%f %f\n", pt.x + pt.normal.x * 100, pt.y + pt.normal.y * 100);
            fprintf(fd, "\n");
        }
    }
    fprintf(fd, "EOD\n");
}

