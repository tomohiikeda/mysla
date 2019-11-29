#pragma once

#include <stdint.h>
#include "PointCloud.hpp"

class IPlotter{
    public:
        virtual bool open(void) = 0;
        virtual void close(void) = 0;
        virtual void plot(const PointCloud& pc) = 0;

    protected:

    private:
};

