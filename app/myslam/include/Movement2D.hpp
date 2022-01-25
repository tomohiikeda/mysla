#pragma once
#include "Common.hpp"
#include "Util.hpp"
#include "Eigen/Core"

using MoveMatrix = Eigen::Matrix<double, 3, 3>;

struct Movement2D {
    public:
        Movement2D(void)
        {
            move_matrix << 1, 0, 0,
                           0, 1, 0,
                           0, 0, 1;
            rotate = 0.0f;
        };

        Movement2D(double x, double y, double theta)
        {
            move_matrix << std::cos(theta), -std::sin(theta), x,
                           std::sin(theta), std::cos(theta), y,
                           0, 0, 1;
            rotate = 0.0f;
        };

        void move(const Movement2D movement)
        {
            this->move_matrix = movement.move_matrix * this->move_matrix;
            this->rotate += movement.rotate;
        };

        MoveMatrix move_matrix;
        double rotate;
};
