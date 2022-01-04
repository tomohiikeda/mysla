#pragma once
#include "Common.hpp"

typedef struct {
    int16_t left;
    int16_t right;
} odometory_t;

class IOdometer{
    public:
        virtual bool init(void) = 0;
        virtual void deinit(void) = 0;
        virtual bool get_odometory(odometory_t *odom) = 0;
        virtual bool get_odometory(int16_t *left, int16_t *right) = 0;
        
    protected:
    private:
};
