#ifndef IMU_GY521_H
#define IMU_GY521_H

#include "stm32f4xx_hal.h"
#include <cstdint>

// MPU-6050 on a GY-521 breakout. In this build it is mounted on the UPPER ARM and gives the
// upper-arm inclination (gravity reference); the elbow joint angle comes from the encoders.
class IMU_GY521 {
public:
    IMU_GY521(I2C_HandleTypeDef* hi2c, uint16_t addr);

    bool begin();                               // reset, WHO_AM_I check, 44 Hz DLPF, +/-2 g, +/-250 dps
    bool whoAmI(uint8_t& id);
    // accel in g, gyro in deg/s (bias-corrected). Auto re-initialises the I2C peripheral after 3 failures.
    bool readSensor(float& ax, float& ay, float& az, float& gx, float& gy, float& gz);
    bool calibrateGyro(uint16_t samples = 200); // arm must be perfectly still
    float gyroBiasZ() const { return bias_z_; }

private:
    I2C_HandleTypeDef* hi2c_;
    uint16_t dev_addr_;
    float bias_x_, bias_y_, bias_z_;
    uint8_t errors_;

    bool readRaw(float& ax, float& ay, float& az, float& gx, float& gy, float& gz);
    bool writeReg(uint8_t reg, uint8_t val);
};

#endif // IMU_GY521_H
