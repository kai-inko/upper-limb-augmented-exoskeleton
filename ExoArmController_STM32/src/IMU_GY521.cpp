#include "IMU_GY521.h"

#define MPU6050_PWR_MGMT_1   0x6B
#define MPU6050_ACCEL_XOUT_H 0x3B

IMU_GY521::IMU_GY521(I2C_HandleTypeDef* hi2c, uint8_t addr) 
    : hi2c_(hi2c), dev_addr_(addr) {}

bool IMU_GY521::begin() {
    uint8_t data = 0x00; // Wake up MPU-6050
    HAL_StatusTypeDef status = HAL_I2C_Mem_Write(hi2c_, dev_addr_, MPU6050_PWR_MGMT_1, 1, &data, 1, HAL_MAX_DELAY);
    return (status == HAL_OK);
}

bool IMU_GY521::readSensor(float &accelX, float &accelY, float &accelZ, float &gyroX, float &gyroY, float &gyroZ) {
    uint8_t rawData[14];
    if (HAL_I2C_Mem_Read(hi2c_, dev_addr_, MPU6050_ACCEL_XOUT_H, 1, rawData, 14, 100) != HAL_OK) {
        return false;
    }

    int16_t rawAccX = (int16_t)((rawData[0] << 8) | rawData[1]);
    int16_t rawAccY = (int16_t)((rawData[2] << 8) | rawData[3]);
    int16_t rawAccZ = (int16_t)((rawData[4] << 8) | rawData[5]);

    int16_t rawGyrX = (int16_t)((rawData[8] << 8) | rawData[9]);
    int16_t rawGyrY = (int16_t)((rawData[10] << 8) | rawData[11]);
    int16_t rawGyrZ = (int16_t)((rawData[12] << 8) | rawData[13]);

    accelX = rawAccX / 16384.0f;
    accelY = rawAccY / 16384.0f;
    accelZ = rawAccZ / 16384.0f;

    gyroX = rawGyrX / 131.0f;
    gyroY = rawGyrY / 131.0f;
    gyroZ = rawGyrZ / 131.0f;

    return true;
}