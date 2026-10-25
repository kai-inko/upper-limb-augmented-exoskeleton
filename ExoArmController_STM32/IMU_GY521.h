#ifndef IMU_GY521_H
#define IMU_GY521_H

#include <Arduino.h>
#include <stdint.h>
#include <Wire.h>   // STM32duino I2C (Wire.setSDA/setSCL available)

// MPU6050 I2C Default Address
#define MPU6050_I2C_ADDR       0x68

// Register Map
#define MPU6050_REG_PWR_MGMT_1 0x6B
#define MPU6050_REG_PWR_MGMT_2 0x6C
#define MPU6050_REG_ACCEL_CFG  0x1C
#define MPU6050_REG_GYRO_CFG   0x1B
#define MPU6050_REG_MOT_THR    0x1F
#define MPU6050_REG_MOT_DUR    0x20
#define MPU6050_REG_INT_PIN_CFG 0x37
#define MPU6050_REG_INT_ENABLE 0x38
#define MPU6050_REG_INT_STATUS 0x3A
#define MPU6050_REG_ACCEL_XOUT 0x3B

class IMU_GY521 {
public:
    IMU_GY521(uint8_t i2cAddr = MPU6050_I2C_ADDR);

    // Initialize I2C communication and configure hardware registers
    bool begin();

    // Configure MPU6050 INT pin & motion/data-ready interrupt generator
    bool configureHardwareInterrupt(bool enableMotionDetection = true, uint8_t motionThreshold = 20);

    // Calibrate gyroscope zero-rate bias
    void calibrateGyro(int samples = 150);

    // Update fused joint angle & raw inertial vector
    bool update();

    // Clear latched interrupt status from MPU6050 register 0x3A
    uint8_t readAndClearInterruptStatus();

    // State & Vector Getters
    float getAngleDeg() const { return filteredAngleDeg; }
    float getAngularVelocityDegPerSec() const { return currentGyroRate; }
    bool  isHealthy() const { return sensorConnected; }

    // Acceleration vector in units of gravity (g)
    float getAccelX_g() const { return accelX_g; }
    float getAccelY_g() const { return accelY_g; }
    float getAccelZ_g() const { return accelZ_g; }

private:
    uint8_t address;
    bool    sensorConnected;

    float gyroBiasY;
    float filteredAngleDeg;
    float currentGyroRate;

    // Measured acceleration vector (in g)
    float accelX_g;
    float accelY_g;
    float accelZ_g;

    unsigned long lastUpdateTimeMicros;
    bool  isFirstRun;

    bool readRawData(int16_t &ax, int16_t &ay, int16_t &az, int16_t &gx, int16_t &gy, int16_t &gz);
};

#endif // IMU_GY521_H
