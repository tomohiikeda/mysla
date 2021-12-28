#include "Common.hpp"
#include "DualShock4.hpp"
#include <unistd.h>
#include <fcntl.h>
#include <linux/joystick.h>
#include <linux/input.h>
#include <iostream>

#define BUTTON_DATA_MAX 100
#define STICK_DATA_MAX 100

DualShock4::DualShock4(void)
{
}

DualShock4::~DualShock4(void)
{
    if (this->fd) {
        close(this->fd);
    }
}

bool DualShock4::init(void)
{
    int err = 0;

    this->fd = open("/dev/input/js0", O_RDONLY);
    if (this->fd < 0) {
        printf("failed to open /dev/input/js0\n");
        goto err_1;
    }
    
    err = pthread_create(&scan_thread, NULL, DualShock4::thread_entry, this);
    if(err){
        std::cout << "failed to pthread_create errno" << err << std::endl;
        goto err_0;
    }

    return true;
    
err_0:
    close(this->fd);
err_1:
    return false;
}

void DualShock4::deinit(void)
{
    this->running = false;
    if (this->fd) {
        close(this->fd);
    }
}

void DualShock4::process_loop(void)
{
    this->running = true;
    
    while (this->running) {
        struct js_event event;
        ssize_t sz_rd = read(fd, &event, sizeof(struct js_event));
        
        if (sz_rd < 0)
            printf("failed to read\n");
        
        if ((size_t)sz_rd >= sizeof(struct js_event)) {
            switch (event.type & ~JS_EVENT_INIT) {
            case JS_EVENT_BUTTON:
                switch (event.number) {
                case 0: on_button_batsu(event.value == 1 ? true: false); break;
                case 1: on_button_maru(event.value == 1 ? true: false); break;
                case 2: on_button_sankaku(event.value == 1 ? true: false); break;
                case 3: on_button_shikaku(event.value == 1 ? true: false); break;
                case 4: on_button_l1(event.value == 1 ? true: false); break;
                case 5: on_button_r1(event.value == 1 ? true: false); break;
                case 6: on_button_l2(event.value == 1 ? true: false); break;
                case 7: on_button_r2(event.value == 1 ? true: false); break;
                case 8: on_button_share(event.value == 1 ? true: false); break;
                case 9: on_button_options(event.value == 1 ? true: false); break;
                case 10: on_button_ps(event.value == 1 ? true: false); break;
                case 11: on_button_l3(event.value == 1 ? true: false); break;
                case 12: on_button_r3(event.value == 1 ? true: false); break;
                default:; break;
                }
            case JS_EVENT_AXIS:
                switch (event.number) {
                case 0: on_axis_l3_lr(event.value); break;
                case 1: on_axis_l3_ud(event.value); break;
                case 2: on_axis_l2(event.value); break;
                case 3: on_axis_r3_lr(event.value); break;
                case 4: on_axis_r3_ud(event.value); break;
                case 5: on_axis_r2(event.value); break;
                case 6: on_axis_lr(event.value); break;
                case 7: on_axis_ud(event.value); break;
                default:; break;
                }
           ; break;
            }
        }
    }
}

/**
 * @brief スレッドエントリ関数
 */
void *DualShock4::thread_entry(void *arg)
{
    ((DualShock4*)arg)->process_loop();
    return NULL;
}
