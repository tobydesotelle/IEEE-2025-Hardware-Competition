#include "macros.h"
#include"ros_functions.h"

void intiLidar();
void intiLidar2();
void initLidar3();
//void setSensorAddress(Adafruit_VL53L1X &sensor, uint8_t newAddress);
float getLidarData(int lidar_num);
void resetAllSensors();
void scanI2CDevices();
void testRead();
void resetSensor(uint8_t i);
void publishLidarData();