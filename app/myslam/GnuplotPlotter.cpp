#include <stdio.h>
#include <algorithm>
#include <fstream>
#include "GnuplotPlotter.hpp"

bool GnuplotPlotter::open(void)
{
    fd = popen("gnuplot", "w");
    if (fd == NULL)
        return false;

    fprintf(fd, "set xr[-5000:5000]\n");
    fprintf(fd, "set yr[-5000:5000]\n");
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

void GnuplotPlotter::plot(const PointCloud *pc) const
{
    if (fd == NULL)
        return;

    const char *plotfile = "/tmp/plot.dat";
    this->pc_to_tmpfile(pc, plotfile);

    fprintf(fd, "plot \"%s\" with points pointtype 7 pointsize 0.2\n", plotfile);
    fflush(fd);
    return;
}

void GnuplotPlotter::plot(const PointCloud *pc_0, const PointCloud *pc_1) const
{
    if (fd == NULL)
        return;

    const char *plotfile_0 = "/tmp/plot_0.dat";
    const char *plotfile_1 = "/tmp/plot_1.dat";
    this->pc_to_tmpfile(pc_0, plotfile_0);
    this->pc_to_tmpfile(pc_1, plotfile_1);

    fprintf(fd, "plot \"%s\",\"%s\" with points pointtype 7 pointsize 0.2\n",
            plotfile_0, plotfile_1);
    fflush(fd);
    return;
}

void GnuplotPlotter::plot(const PointCloud *pc_0,
                          const PointCloud *pc_1,
                          const std::vector<uint32_t>& associate_list) const
{
    if (fd == NULL)
        return;

    const char *plotfile_0 = "/tmp/plot_0.dat";
    const char *plotfile_1 = "/tmp/plot_1.dat";
    const char *plotfile_2 = "/tmp/segments.dat";
    this->pc_to_tmpfile(pc_0, plotfile_0);
    this->pc_to_tmpfile(pc_1, plotfile_1);
    this->associate_to_tmpfile(pc_0, pc_1, associate_list, plotfile_2);

    fprintf(fd, "plot \"%s\",\"%s\" with points pointtype 7 pointsize 0.2, \"%s\" w lp\n",
            plotfile_0, plotfile_1, plotfile_2);
    fflush(fd);
    return;
}

void GnuplotPlotter::pc_to_tmpfile(const PointCloud *pc, const char *plotfile) const
{
    std::ofstream ofs(plotfile);
    for (size_t i = 0; i < pc->size(); i++) {
        ofs << pc->at(i).x << " " << pc->at(i).y << std::endl;
    }
    ofs.close();
}

void GnuplotPlotter::associate_to_tmpfile(const PointCloud *cur_pc, const PointCloud *ref_pc,
            const std::vector<uint32_t>& associate_list, const char *filename) const
{
    std::ofstream ofs(filename);
    for (size_t i = 0; i < associate_list.size(); i++) {
        Point cur_point = cur_pc->at(i);
        Point ref_point = ref_pc->at(associate_list.at(i));
        ofs << cur_point.x << " " << cur_point.y << std::endl;
        ofs << ref_point.x << " " << ref_point.y << std::endl;
        ofs << std::endl;
    }
    ofs.close();
}

