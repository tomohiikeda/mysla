#pragma once

#include "IOdometer.hpp"

class PulseSensor : public IOdometer{
    public:
        bool init(void){ return true; }
        bool start(void){ return true; }
        void stop(void){}    
    
    protected:

    private:
};
