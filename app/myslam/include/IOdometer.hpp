#pragma once
#include "Common.hpp"

class IOdometer{
    public:
        virtual bool init(void) = 0;
        virtual bool start(void) = 0;
        virtual void stop(void) = 0;
        virtual bool get_odometory(int16_t *od_l, int16_t *od_r) = 0;
        
    protected:
    private:
};
