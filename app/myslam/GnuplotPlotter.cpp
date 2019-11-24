#include <stdio.h>
#include <algorithm>
#include <fstream>
#include "GnuplotPlotter.hpp"

void GnuplotPlotter::open(void)
{
    fd = popen("gnuplot", "w");
    return;
}

void GnuplotPlotter::close(void)
{
    if(fd == NULL) return;

    pclose(fd);
    fd = NULL;
    return;
}

void GnuplotPlotter::plot(PointCloud pc)
{
    if(fd == NULL) return;
    
    const char *plotfile = "/tmp/plot.dat";
    std::ofstream ofs(plotfile);
    for(int i=0; i<pc.size(); i++){
        ofs << pc.at(i).x << " " << pc.at(i).y << std::endl;
    }
    ofs.close();

    fprintf(fd, "plot \"%s\"\n", plotfile);
    fflush(fd);
    return;
}
