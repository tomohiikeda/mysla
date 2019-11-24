#include <stdio.h>
#include <algorithm>
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

static int x = 0;
void GnuplotPlotter::plot(PointCloud pc)
{
    if(fd == NULL) return;
    
    fprintf(fd, "plot sin(%d*x)\n", x);
    fflush(fd);
    x++;
    return;
}
