#pragma once

class IMotor{
    public:
        virtual bool init(void) = 0;
        virtual void deinit(void) = 0;
        virtual void move_front(double mm, double speed) = 0;
        virtual void turn_left(double rad, double speed) = 0;

    protected:
    private:
};
