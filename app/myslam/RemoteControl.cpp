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
    this->current_r3_lr = -value;
    set_motor_freq(this->current_l3_ud, this->current_r3_lr);
}

void RemoteControl::on_axis_l3_ud(int16_t value)
{
    this->current_l3_ud = -value;
    set_motor_freq(this->current_l3_ud, this->current_r3_lr);
}

void RemoteControl::set_motor_freq(int16_t ud_val, int16_t lr_val)
{
    int16_t base = (double)ud_val * 400 / 32768;
    int16_t offset = (double)lr_val * 400 / 32768;
    int16_t freq_r = base;
    int16_t freq_l = base;

    if (base < 0)
        offset = -offset;

    if (lr_val >= 0)
        freq_r += offset;
    else 
        freq_l += -offset;

    printf("ud_val=%d, lr_val=%d, l:%d r:%d\n", ud_val, lr_val, freq_l, freq_r);
    motor.set_freq(freq_l, freq_r);
}
