#pragma once

class IOdometer{
    public:
        virtual bool init(void) = 0;
        virtual bool start(void) = 0;
        virtual void stop(void) = 0;

    protected:
    private:
};
