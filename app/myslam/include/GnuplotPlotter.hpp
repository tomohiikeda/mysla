#pragma once

#include <stdint.h>
#include "Plotter.hpp"
#include "PointCloud.hpp"

class GnuplotPlotter: public Plotter{
    public:
        void open(void);
        void close(void);
        void plot(PointCloud pc);

    protected:
        FILE *fd;

    private:
};
