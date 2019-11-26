#pragma once

#include <stdint.h>
#include "PointCloud.hpp"

class Plotter{
    public:
        virtual void open(void) = 0;
        virtual void close(void) = 0;
        virtual void plot(PointCloud pc) = 0;

    protected:

    private:
};

