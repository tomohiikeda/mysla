#pragma once

#include <stdint.h>

class Plotter{
    public:
        virtual void open(void) = 0;
        virtual void close(void) = 0;
        virtual void plot(void) = 0;
        
    protected:

    private:
};

