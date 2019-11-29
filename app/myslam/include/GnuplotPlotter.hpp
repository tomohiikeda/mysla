#pragma once

#include <stdint.h>
#include "IPlotter.hpp"
#include "PointCloud.hpp"

class GnuplotPlotter: public IPlotter{
    public:
        bool open(void);
        void close(void);
        void plot(const PointCloud& pc);

    protected:
        FILE *fd;

    private:
};
