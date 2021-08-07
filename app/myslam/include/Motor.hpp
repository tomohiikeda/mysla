#pragma once
#include "Common.hpp"
#include "IMotor.hpp"

class Motor : public IMotor{
    public:
        bool init(void);
        void deinit(void);
        void move_front(double mm, double speed);
        void move_front(int16_t freq);
        void turn_left(double rad, double speed);
        void turn_left(int16_t freq);

    protected:
        FILE *fd[2];

    private:
};
