#pragma once

#include "IPlotter.hpp"
#include "SlamData.hpp"
#include "DataRetriever.hpp"

class Slam {
    public:

        Slam(IPlotter& plotter, DataRetriever& retriever):
            plotter(plotter),
            retriever(retriever) {
            running = false;
        }

        bool init(void);
        bool start(void);
        void stop(void);
        void debug_on(void) {
            this->debug = true;
        }

    protected:
        IPlotter& plotter;
        DataRetriever& retriever;

        bool debug = false;
        bool running;
        pthread_t slam_thread;
        void process_loop(void);
        static void *thread_entry(void *arg);
};
