#pragma once

#include <mutex>
#include "Common.hpp"
#include "ISensor.hpp"
#include "IOdometer.hpp"
#include "SlamData.hpp"

class DataRetriever {
    public:
        typedef enum {
            offline_mode,
            online_mode,
        } slam_mode_e;

        typedef struct {
            ISensor& sensor;        // for online
            IOdometer& odometer;    // for online
            std::string dir_name;   // for offline
        } retrieve_info_t;

        DataRetriever(const slam_mode_e mode,
                      ISensor &sensor,
                      IOdometer &odometer,
                      std::string dir_name,
                      uint32_t start_index,
                      uint32_t end_index) : sensor(sensor), odometer(odometer)
        {
            this->mode = mode;
            this->dir_name = dir_name;
            this->file_index = start_index;
            this->end_index = end_index;
        }

        bool init(void);
        bool start(void);
        void stop(void);
        bool retrieve(SlamData& slam_data);

    protected:
        slam_mode_e mode;
        ISensor& sensor;
        IOdometer& odometer;
        std::string dir_name;
        uint32_t file_index;
        uint32_t end_index;

        bool retrieve_online(SlamData& slam_data);
        bool retrieve_offline(SlamData& slam_data);
        std::vector<SlamData> data_fifo;
        std::mutex mtx_;

        bool running;
        pthread_t thread;
        void process_loop(void);
        static void *thread_entry(void *arg);
};
