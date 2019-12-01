#include <stdio.h>
#include <stddef.h>
#include <cmath>
#include <cstdlib>
#include <string.h>
#include "rplidar.h"
#include "Lidar.hpp"
#include "PointCloud.hpp"

#define _countof(_Array) (int)(sizeof(_Array) / sizeof(_Array[0]))

/**
 * @brief Lidar初期化関数
 */
bool Lidar::init(void)
{
    u_result result = RESULT_OK;

    _drv = RPlidarDriver::CreateDriver(DRIVER_TYPE_SERIALPORT);
    if(!_drv) {
        fprintf(stderr, "failed to create driver\n");
        goto error;
    }

    result = _drv->connect("/dev/ttyUSB0", 115200);
    if(IS_FAIL(result)) {
        fprintf(stderr, "failed to connect result=0x%x\n", result);
        goto error;
    }

    if(get_devinfo() == false)
        goto error;

    if(check_health() == false)
        goto error;

    return true;

error:
    if(_drv != NULL){
        RPlidarDriver::DisposeDriver(_drv);
        _drv = NULL;
    }
    return false;
}


/**
 * @brief 計測開始
 */
bool Lidar::start(void)
{
    u_result result = RESULT_OK;

    if(_drv == NULL) {
        fprintf(stderr, "can not start, driver is not created\n");
        return false;
    }

    result = _drv->startMotor();
    if (IS_FAIL(result)) {
        printf("failed to startMotor result=%x\n", result);
        return false;
    }

    result = _drv->startScan(0, 1);
    if (IS_FAIL(result)) {
        printf("failed to startScan result=%x\n", result);
        return false;
    }

    return true;
}


/**
 * @brief 計測終了
 */
void Lidar::stop(void)
{
    _drv->stop();
    _drv->stopMotor();
}

/**
 * @brief 計測終了
 */
bool Lidar::get_point_cloud(PointCloud& point_cloud)
{
    rplidar_response_measurement_node_hq_t nodes[8192];
    size_t count = _countof(nodes);
    
    u_result result = _drv->grabScanDataHq(nodes, count);
    if (IS_FAIL(result)) {
        printf("failed to grabScanDataHq result=%x\n", result);
        return false;
    }

    result = _drv->ascendScanData(nodes, count);
    if (IS_FAIL(result)) {
        printf("failed to ascendScanData result=%x\n", result);
        return false;
    }

    point_cloud.clear();
    const double pi = 3.141592653589793;
    for(int i=0; i<(int)count; i++){
        double deg = nodes[i].angle_z_q14 * 90.f / 16384.f;
        double dist = nodes[i].dist_mm_q2 / 4.0f;
        double x = dist * std::sin(deg * pi / 180);
        double y = dist * std::cos(deg * pi / 180);
        if(dist != 0){
            Point p(x, y);
            point_cloud.add(p);
        }
    }

    return true;
}

/**
 * @brief 健康診断
 * @return true:健康、false:不健康
 */
bool Lidar::check_health(void)
{
    rplidar_response_device_health_t healthinfo;
    u_result result = _drv->getHealth(healthinfo);

    if(IS_OK(result)) {
        printf("RPLidar health status : %d\n", healthinfo.status);
        if(healthinfo.status == RPLIDAR_STATUS_ERROR) {
            fprintf(stderr, "Error, rplidar internal error detected. Please "
                            "reboot the device to retry.\n");
            // enable the following code if you want rplidar to be reboot by
            // software drv->reset();
            return false;
        } else {
            return true;
        }
    } else {
        fprintf(stderr, "Error, cannot retrieve the lidar health code: %x\n",
                result);
        return false;
    }
}

/**
 * @brief デバイス情報を取得してコンソールに表示
 * @return true: 取得成功、false: 取得失敗
 */
bool Lidar::get_devinfo(void)
{
    rplidar_response_device_info_t devinfo;
    u_result result = _drv->getDeviceInfo(devinfo);
    if(IS_FAIL(result)) {
        fprintf(stderr, "failed to getDeviceInfo result=0x%x\n", result);
        return false;
    }

    printf("RPLIDAR S/N: ");
    for(int pos = 0; pos < 16; ++pos) {
        printf("%02X", devinfo.serialnum[pos]);
    }

    printf("\n"
           "Firmware Ver: %d.%02d\n"
           "Hardware Rev: %d\n",
           devinfo.firmware_version >> 8, devinfo.firmware_version & 0xFF,
           (int)devinfo.hardware_version);

    std::vector<RplidarScanMode> supportedModes;
    result = _drv->getAllSupportedScanModes(supportedModes);
    if(IS_FAIL(result)) {
        fprintf(stderr, "failed to getAllSupportedScanModes result=0x%x\n",
                result);
        return false;
    }

    for(size_t i=0; i<supportedModes.size(); i++){
        printf("%s,  samplerate=%f\n", 
                supportedModes.at(i).scan_mode,
                supportedModes.at(i).us_per_sample);
    }
    return true;
}
