#include <stdio.h>
#include <algorithm>
#include <fstream>
#include "GnuplotPlotter.hpp"

bool GnuplotPlotter::open(void)
{
    fd = popen("gnuplot", "w");
    if (fd == NULL)
        return false;
    
    fprintf(fd, "set xr[-3500:3500]\n");
    fprintf(fd, "set yr[-3500:3500]\n");
    fprintf(fd, "set size square\n");
    fflush(fd);
    return true;
}

void GnuplotPlotter::close(void)
{
    if(fd == NULL) return;

    pclose(fd);
    fd = NULL;
    return;
}

void GnuplotPlotter::plot(const PointCloud& pc)
{
    if(fd == NULL) return;
    
    const char *plotfile = "/tmp/plot.dat";
    std::ofstream ofs(plotfile);
    for(size_t i=0; i<pc.size(); i++){
        ofs << pc.at(i).x << " " << pc.at(i).y << std::endl;
    }
    ofs.close();

    fprintf(fd, "plot \"%s\" with points pointtype 7 pointsize 0.2\n", plotfile);
    fflush(fd);
    return;
}
