#pragma once
#include <pthread.h> 

class RemoteController {
    public:
        bool init(void);
        void deinit(void);

    protected:
        int fd;
        bool running;
        pthread_t scan_thread;
        static void *thread_entry(void *arg);
        void process_loop(void);

    private:
};
