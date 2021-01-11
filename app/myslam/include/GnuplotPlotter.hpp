#pragma once

#include <stdint.h>
#include "IPlotter.hpp"
#include "PointCloud.hpp"

class GnuplotPlotter: public IPlotter{
    public:
        bool open(void);
        void close(void);
        void plot(const PointCloud* pc) const;
        void plot(const PointCloud* pc0, const PointCloud* pc1) const;
        void plot(const PointCloud *pc_0,
                  const PointCloud *pc_1,
                  const std::vector<uint32_t>& associate_list) const;

    protected:
        FILE *fd;
        void pc_to_tmpfile(const PointCloud *pc, const char *plotfile) const;
        void associate_to_tmpfile(const PointCloud *cur_pc, const PointCloud *ref_pc,
            const std::vector<uint32_t>& associate_list, const char *filename) const;

    private:
};
