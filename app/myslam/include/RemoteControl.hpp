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

    private:
};
