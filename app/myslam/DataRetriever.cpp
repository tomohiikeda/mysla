#include "DataRetriever.hpp"

bool DataRetriever::init(void)
{
    if (this->mode == online_mode) {
        if (this->sensor.init() == false)
            return false;
        if (this->odometer.init() == false)
            return false;
    }

    return true;
}

bool DataRetriever::start(void)
{
    if (this->mode == online_mode) {

        if (this->sensor.start() == false)
            return false;

        int err = pthread_create(&thread, NULL, DataRetriever::thread_entry, this);
        if(err){
            std::cout << "failed to pthread_create errno" << err << std::endl;
            return false;
        }
    }

    return true;
}

void DataRetriever::stop(void)
{
    if (this->mode == online_mode) {
        running = false;
        pthread_join(thread, NULL);
        this->sensor.stop();
        this->odometer.deinit();
    }
}

bool DataRetriever::retrieve(SlamData& slam_data)
{
    if (this->mode == DataRetriever::offline_mode)
        return retrieve_offline(slam_data);
    else
        return retrieve_online(slam_data);
}

bool DataRetriever::retrieve_online(SlamData& slam_data)
{
    if (this->data_fifo.size() == 0)
        return false;

    slam_data = this->data_fifo[0];
    this->data_fifo.erase(this->data_fifo.begin());

    return true;
}

bool DataRetriever::retrieve_offline(SlamData& slam_data)
{
    if (this->file_index > this->end_index)
        return false;

    char filename[50];
    sprintf(filename, "%s/pt_%04d.txt", this->dir_name.c_str(), this->file_index);

    if (slam_data.load_from_file(filename) == false)
        return false;

    slam_data.pc()->analyse_points();
    this->file_index++;
    return true;
}

/**
 * @brief SLAMの実行スレッド
 */
void DataRetriever::process_loop(void)
{
    this->running = true;

    while (this->running == true) {

        SlamData slam_data;
        slam_data.timestamp = get_currenttime();

        if (this->sensor.get_point_cloud(slam_data.pc()) == false)
            break;

        if (this->odometer.get_odometory(slam_data.odometory()) == false)
            break;

        this->data_fifo.push_back(slam_data);

        wait_for_time(slam_data.timestamp, 0.5f);
    }
}

/**
 * @brief スレッドエントリ関数
 */
void *DataRetriever::thread_entry(void *arg)
{
    ((DataRetriever*)arg)->process_loop();
    return NULL;
}
