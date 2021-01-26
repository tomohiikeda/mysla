#include <stdio.h>
#include "RemoteController.hpp"
#include <unistd.h>
#include <fcntl.h>
#include <linux/joystick.h>
#include <linux/input.h>
#include <iostream>

#define BUTTON_DATA_MAX 100
#define STICK_DATA_MAX 100

bool RemoteController::init(void)
{
    int err = 0;

    this->fd = open("/dev/input/js0", O_RDONLY);
    printf("fd=%d", this->fd);
    if (this->fd < 0) {
        printf("failed to open /dev/input/js0\n");
        goto err_1;
    }
    
    err = pthread_create(&scan_thread, NULL, RemoteController::thread_entry, this);
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

void RemoteController::deinit(void)
{
    running = false;
    pthread_join(scan_thread, NULL);
    if (this->fd) {
        close(this->fd);
    }
}

void RemoteController::process_loop(void)
{
    unsigned char  ButtonData[BUTTON_DATA_MAX];
    signed int     StickData[STICK_DATA_MAX];    
    running = true;
    
    while (this->running) {
        struct js_event  event;
        if (read(fd, &event, sizeof(struct js_event)) >= sizeof(struct js_event)) {
            switch (event.type & 0x7f) {
            case JS_EVENT_BUTTON:
                if (event.number < BUTTON_DATA_MAX) {
                    ButtonData[event.number] = event.value != 0;
                    std::cout << event.value << std::endl;
                }
                break;
            case JS_EVENT_AXIS:
                if(event.number < STICK_DATA_MAX) {
                    StickData[ event.number ]= event.value;
                }
            break;
            }
        }
    }
}

/**
 * @brief スレッドエントリ関数
 */
void *RemoteController::thread_entry(void *arg)
{
    ((RemoteController*)arg)->process_loop();
    return NULL;
}