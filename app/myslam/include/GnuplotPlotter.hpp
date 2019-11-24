#pragma once

#include <stdint.h>
#include "Plotter.hpp"

class GnuplotPlotter: public Plotter{
    public:
        void open(void);
        void close(void);
        void plot(void);

    protected:
        FILE *fd;

    private:
};
