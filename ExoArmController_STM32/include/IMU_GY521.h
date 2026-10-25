#ifndef IMU_GY521_H
#define IMU_GY521_H

#include "stm32f1xx_hal.h"
#include <cstdint>

class IMU_GY521 {
public:
    IMU_GY521(I2C_HandleTypeDef* hi2c, uint8_t addr);
    bool begin();
    bool readSensor(float &accelX, float &accelY, float &accelZ, float &gyroX, float &gyroY, float &gyroZ);

private:
    I2C_HandleTypeDef* hi2c_;
    uint8_t dev_addr_;
};

#endif // IMU_GY521_H