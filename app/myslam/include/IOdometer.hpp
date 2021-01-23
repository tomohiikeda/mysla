#pragma once

class IOdometer{
    public:
        virtual bool init(void) = 0;
        virtual bool start(void) = 0;
        virtual void stop(void) = 0;
        virtual void get_odometory(double *od_r, double *od_l) = 0;
        
    protected:
    private:
};
