#ifndef IMU_GY521_H
#define IMU_GY521_H

#include <Arduino.h>
#include <Wire.h>

// GY-521 / MPU6050 Default I2C Address (AD0 pin connected to GND)
#define MPU6050_I2C_ADDR       0x68

// Register Addresses
#define MPU6050_REG_PWR_MGMT_1 0x6B
#define MPU6050_REG_ACCEL_CFG  0x1C
#define MPU6050_REG_GYRO_CFG   0x1B
#define MPU6050_REG_ACCEL_XOUT 0x3B

class IMU_GY521 {
public:
    IMU_GY521(uint8_t i2cAddr = MPU6050_I2C_ADDR);

    // Initialize I2C communication and configure sensor registers
    bool begin();

    // Calibrate gyroscope zero-rate bias (keep arm still during calibration)
    void calibrateGyro(int samples = 150);

    // Read raw sensor data and update fused elbow joint angle via Complementary Filter
    bool update();

    // Getters
    float getAngleDeg() const { return filteredAngleDeg; }
    float getAngularVelocityDegPerSec() const { return currentGyroRate; }
    bool  isHealthy() const { return sensorConnected; }

    // Manually override/tare the joint zero angle
    void setAngleOffset(float offsetDeg) { angleOffsetDeg = offsetDeg; }
    float getAngleOffset() const { return angleOffsetDeg; }

private:
    uint8_t address;
    bool    sensorConnected;

    // Calibration bias
    float gyroBiasY;
    float angleOffsetDeg;

    // Filter memory
    float filteredAngleDeg;
    float currentGyroRate;
    unsigned long lastUpdateTimeMicros;
    bool  isFirstRun;

    // Read helper
    bool readRawData(int16_t &ax, int16_t &ay, int16_t &az, int16_t &gx, int16_t &gy, int16_t &gz);
};

#endif // IMU_GY521_H
