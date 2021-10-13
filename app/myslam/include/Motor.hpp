#pragma once
#include "Common.hpp"
#include "IMotor.hpp"

class Motor : public IMotor{
    public:
        bool init(void);
        void deinit(void);
        void move_front(double mm, double speed);
        void move_front(double speed);
        void move_front(int16_t freq);
        void turn_left(double rad, double speed);
        void turn_left(int16_t freq);
        void set_freq(int16_t freq_l, int16_t freq_r);

    protected:
        FILE *fd[2];
        int16_t speed_to_freq(double speed);
        void write_(int16_t freq_l, int16_t freq_r);

    private:
};
