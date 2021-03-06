#pragma once

#include <stdint.h>
#include "PointCloud.hpp"

class IPlotter{
    public:
        virtual bool open(void) = 0;
        virtual void close(void) = 0;
        virtual void plot(const PointCloud *pc) const = 0;
        virtual void plot(const PointCloud* pc0, const PointCloud* pc1) const = 0;
        virtual void plot(const PointCloud *pc_0,
                          const PointCloud *pc_1,
                          const std::vector<uint32_t>& associate_list) const = 0;

    protected:

    private:
};

