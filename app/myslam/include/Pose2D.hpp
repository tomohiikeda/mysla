#pragma once
#include "Common.hpp"
#include "Util.hpp"

class Pose2D {
    public:
        double x;
        double y;
        double direction;
        Pose2D(void);
        Pose2D(double x, double y, double dir);
        void print(void) const;
        void print(uint32_t num) const;
        void move_to(const Pose2D& move);
        void set(double x, double y, double dir);
        void save_to_file(const std::string filename) const;
        void load_from_file(const std::string filename);

    protected:

    private:
};
