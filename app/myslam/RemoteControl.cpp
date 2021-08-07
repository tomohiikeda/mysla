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
    printf("maru %d\n", on);
}

void RemoteControl::on_axis_r3_lr(int16_t value)
{
    printf("r3 lr %d\n", value);
    motor.turn_left((int32_t)((double)-value*400/32768));
}

void RemoteControl::on_axis_l3_ud(int16_t value)
{
    printf("r3 ud %d\n", value);
    motor.move_front((int32_t)((double)-value*500/32768));
}
