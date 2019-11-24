#include <stdio.h>
#include "GnuplotPlotter.hpp"

void GnuplotPlotter::open(void)
{
    fd = popen("gnuplot", "w");
    fprintf(fd, "plot 1,1\n");
    fflush(fd);
    return;
}

void GnuplotPlotter::close(void)
{
    if(fd != NULL){
        pclose(fd);
        fd = NULL;
    }
    return;
}

void GnuplotPlotter::plot(void)
{
    if(fd != NULL){
        //fprintf(fd, "plot 1,1\n");
        printf("plot\n");
    }
    return;
}

