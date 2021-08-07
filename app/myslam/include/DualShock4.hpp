#pragma once
#include "Common.hpp"
#include <pthread.h> 

class DualShock4 {
    public:
        DualShock4(void);
        virtual ~DualShock4(void);
        bool init(void);
        void deinit(void);
        virtual void on_button_batsu(bool on) {return;}
        virtual void on_button_maru(bool on) {return;}
        virtual void on_button_sankaku(bool on) {return;}
        virtual void on_button_shikaku(bool on) {return;}
        virtual void on_button_l1(bool on) {return;}
        virtual void on_button_r1(bool on) {return;}
        virtual void on_button_l2(bool on) {return;}
        virtual void on_button_r2(bool on) {return;}
        virtual void on_button_share(bool on) {return;}
        virtual void on_button_options(bool on) {return;}
        virtual void on_button_ps(bool on) {return;}
        virtual void on_button_l3(bool on) {return;}
        virtual void on_button_r3(bool on) {return;}
        virtual void on_axis_l3_lr(int16_t value) {return;}
        virtual void on_axis_l3_ud(int16_t value) {return;}
        virtual void on_axis_l2(int16_t value) {return;}
        virtual void on_axis_r3_lr(int16_t value) {return;}
        virtual void on_axis_r3_ud(int16_t value) {return;}
        virtual void on_axis_r2(int16_t value) {return;}
        virtual void on_axis_lr(int16_t value) {return;}
        virtual void on_axis_ud(int16_t value) {return;}

    protected:
        int fd;
        bool running;
        pthread_t scan_thread;
        static void *thread_entry(void *arg);
        void process_loop(void);
        void kill_thread(void);
    
};

/*
BUTTON_BATSU        0
BUTTON_MARU         1
BUTTON_SANKAKU      2
BUTTON_SHIKAKU      3
BUTTON_L1           4
BUTTON_R1           5
BUTTON_L2           6
BUTTON_R2           7
BUTTON_SHARE        8
BUTTON_OPTIONS      9
BUTTON_PS           10
BUTTON_L3           11
BUTTON_R3           12
AXIS_L3_LR          0
AXIS_L3_UD          1
AXIS_L2             2
AXIS_R3_LR          3
AXIS_R3_UD          4
AXIS_R2             5
AXIS_LR             6
AXIS_UD             7
*/