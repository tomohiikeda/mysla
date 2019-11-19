#include <stdio.h>
#include "Lidar.hpp"
#include "rplidar.h"


bool Lidar::init(const char *devname, const uint32_t baudrate)
{
    u_result result = RESULT_OK;

    this->_drv = RPlidarDriver::CreateDriver(DRIVER_TYPE_SERIALPORT);
    if (!_drv) {
        fprintf(stderr, "failed to create driver\n");
        goto error;
    }

    result = _drv->connect(devname, baudrate);
    if (IS_FAIL(result)){
        fprintf(stderr, "failed to connect result=0x%x, "
                        "devname=%s, baudrate=%d\n",
                    result, devname, baudrate);
        goto error;
    }
    
    rplidar_response_device_info_t devinfo;
    result = _drv->getDeviceInfo(devinfo);
    if (IS_FAIL(result)){
        fprintf(stderr, "failed to getDeviceInfo result=0x%x\n", result);
        goto error;
    }

    printf("RPLIDAR S/N: ");
    for (int pos = 0; pos < 16 ;++pos) {
        printf("%02X", devinfo.serialnum[pos]);
    }

    printf("\n"
            "Firmware Ver: %d.%02d\n"
            "Hardware Rev: %d\n"
            , devinfo.firmware_version>>8
            , devinfo.firmware_version & 0xFF
            , (int)devinfo.hardware_version);

    
    if (check_health() == false){
        goto error;
    }
    
    return true;

error:
    if(_drv != NULL)
        RPlidarDriver::DisposeDriver(_drv);
    
    _drv = NULL;

    return false;
}


void Lidar::start(void)
{
    if(_drv == NULL){
        return;
    }

    _drv->startMotor();
    _drv->startScan(0,1);

/*
        // fetech result and print it out...
    while (1) {
        rplidar_response_measurement_node_t nodes[8192];
        size_t   count = _countof(nodes);

        op_result = drv->grabScanData(nodes, count);

        if (IS_OK(op_result)) {
            drv->ascendScanData(nodes, count);
            for (int pos = 0; pos < (int)count ; ++pos) {
                printf("%s theta: %03.2f Dist: %08.2f Q: %d \n", 
                    (nodes[pos].sync_quality & RPLIDAR_RESP_MEASUREMENT_SYNCBIT) ?"S ":"  ", 
                    (nodes[pos].angle_q6_checkbit >> RPLIDAR_RESP_MEASUREMENT_ANGLE_SHIFT)/64.0f,
                    nodes[pos].distance_q2/4.0f,
                    nodes[pos].sync_quality >> RPLIDAR_RESP_MEASUREMENT_QUALITY_SHIFT);
            }
        }

        if (ctrl_c_pressed){ 
            break;
        }
    }
*/    

}

bool Lidar::check_health(void)
{
    rplidar_response_device_health_t healthinfo;
    u_result op_result = _drv->getHealth(healthinfo);
    
    if (IS_OK(op_result)) {
        printf("RPLidar health status : %d\n", healthinfo.status);
        if (healthinfo.status == RPLIDAR_STATUS_ERROR) {
            fprintf(stderr, "Error, rplidar internal error detected. Please reboot the device to retry.\n");
            // enable the following code if you want rplidar to be reboot by software
            // drv->reset();
            return false;
        } else {
            return true;
        }
    } else {
        fprintf(stderr, "Error, cannot retrieve the lidar health code: %x\n", op_result);
        return false;
    }
}


void Lidar::stop(void)
{
    _drv->stop();
    _drv->stopMotor();
}
