#include "DataRetriever.hpp"

bool DataRetriever::init(void)
{
    if(this->sensor.init() == false)
        return false;
    
    if(this->odometer.init() == false)
        return false;

    return true;
}

bool DataRetriever::start(void)
{
    if (this->mode == online_mode) {
        if(this->sensor.start() == false) {
            return false;
        }
    }

    return true;
}

void DataRetriever::stop(void)
{
    sensor.stop();
    odometer.deinit();
}

bool DataRetriever::retrieve(SlamData& slam_data)
{
    if (this->mode == DataRetriever::offline_mode)
        return retrieve_offline(slam_data);
    else if (this->mode == DataRetriever::online_mode)
        return retrieve_online(slam_data);
}

bool DataRetriever::retrieve_online(SlamData& slam_data)
{
    return false;
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
