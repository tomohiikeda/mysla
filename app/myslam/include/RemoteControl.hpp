#pragma once

#include "Common.hpp"
#include "DualShock4.hpp"
#include "Motor.hpp"

class RemoteControl : public DualShock4 {
    public:
        RemoteControl(Motor& motor) : motor(motor) {}
        void on_button_batsu(bool on);
        void on_button_maru(bool on);
        void on_axis_r3_lr(int16_t value);
        void on_axis_l3_ud(int16_t value);

    protected:
        Motor& motor;
        void set_motor_freq(int16_t left, int16_t right);
        int16_t current_l3_ud = 0;
        int16_t current_r3_lr = 0;

    private:
};
