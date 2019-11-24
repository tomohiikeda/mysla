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
    
    std::ofstream ofs("plot.dat");
    for(int i=0; i<pc.size(); i++){
        ofs << pc.at(i).x << " " << pc.at(i).y << std::endl;
    }
    ofs.close();

    fprintf(fd, "plot \"plot.dat\"\n");
    fflush(fd);
    return;
}
