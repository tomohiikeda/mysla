#include "RemoteControl.hpp"

void RemoteControl::on_button_batsu(bool on)
{
    if (on)
        motor.move_front(700);
    else
        motor.move_front(0);
}

void RemoteControl::on_button_maru(bool on)
{
}

void RemoteControl::on_axis_r3_lr(int16_t value)
{
    motor.turn_left((int32_t)((double)-value*400/32768));
}

void RemoteControl::on_axis_l3_ud(int16_t value)
{
    motor.move_front((int32_t)((double)-value*500/32768));
}
