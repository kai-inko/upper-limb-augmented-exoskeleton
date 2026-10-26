#include "IMU_GY521.h"
#include <cmath>

#define MPU_REG_SMPLRT_DIV   0x19
#define MPU_REG_CONFIG       0x1A
#define MPU_REG_GYRO_CONFIG  0x1B
#define MPU_REG_ACCEL_CONFIG 0x1C
#define MPU_REG_ACCEL_XOUT_H 0x3B
#define MPU_REG_PWR_MGMT_1   0x6B
#define MPU_REG_WHO_AM_I     0x75

#define I2C_TIMEOUT_MS       20

IMU_GY521::IMU_GY521(I2C_HandleTypeDef* hi2c, uint16_t addr)
    : hi2c_(hi2c), dev_addr_(addr), bias_x_(0), bias_y_(0), bias_z_(0), errors_(0) {}

bool IMU_GY521::writeReg(uint8_t reg, uint8_t val) {
    return HAL_I2C_Mem_Write(hi2c_, dev_addr_, reg, I2C_MEMADD_SIZE_8BIT, &val, 1, I2C_TIMEOUT_MS) == HAL_OK;
}

bool IMU_GY521::whoAmI(uint8_t& id) {
    return HAL_I2C_Mem_Read(hi2c_, dev_addr_, MPU_REG_WHO_AM_I, I2C_MEMADD_SIZE_8BIT, &id, 1, I2C_TIMEOUT_MS) == HAL_OK;
}

bool IMU_GY521::begin() {
    uint8_t id = 0;
    if (!whoAmI(id)) return false;
    // 0x68 = MPU-6050; 0x70/0x71/0x72 = common GY-521 clones (MPU-6500 family)
    if (id != 0x68 && id != 0x70 && id != 0x71 && id != 0x72) return false;

    if (!writeReg(MPU_REG_PWR_MGMT_1, 0x80)) return false;   // device reset
    HAL_Delay(100);
    if (!writeReg(MPU_REG_PWR_MGMT_1, 0x01)) return false;   // wake, clock = gyro-X PLL
    HAL_Delay(10);
    bool ok = true;
    ok &= writeReg(MPU_REG_CONFIG,       0x03);              // DLPF ~44 Hz (kills motor/vibration noise)
    ok &= writeReg(MPU_REG_SMPLRT_DIV,   0x00);              // 1 kHz internal sample rate
    ok &= writeReg(MPU_REG_GYRO_CONFIG,  0x00);              // +/-250 dps  -> 131 LSB/(deg/s)
    ok &= writeReg(MPU_REG_ACCEL_CONFIG, 0x00);              // +/-2 g      -> 16384 LSB/g
    return ok;
}

bool IMU_GY521::readRaw(float& ax, float& ay, float& az, float& gx, float& gy, float& gz) {
    uint8_t d[14];
    if (HAL_I2C_Mem_Read(hi2c_, dev_addr_, MPU_REG_ACCEL_XOUT_H, I2C_MEMADD_SIZE_8BIT, d, 14, I2C_TIMEOUT_MS) != HAL_OK)
        return false;

    ax = (int16_t)((d[0]  << 8) | d[1])  / 16384.0f;
    ay = (int16_t)((d[2]  << 8) | d[3])  / 16384.0f;
    az = (int16_t)((d[4]  << 8) | d[5])  / 16384.0f;
    // d[6], d[7] = temperature (skipped)
    gx = (int16_t)((d[8]  << 8) | d[9])  / 131.0f;
    gy = (int16_t)((d[10] << 8) | d[11]) / 131.0f;
    gz = (int16_t)((d[12] << 8) | d[13]) / 131.0f;
    return true;
}

bool IMU_GY521::readSensor(float& ax, float& ay, float& az, float& gx, float& gy, float& gz) {
    if (!readRaw(ax, ay, az, gx, gy, gz)) {
        if (++errors_ >= 3) {                 // bus lock-up recovery: reset the I2C peripheral
            HAL_I2C_DeInit(hi2c_);
            HAL_I2C_Init(hi2c_);
            errors_ = 0;
        }
        return false;
    }
    errors_ = 0;
    gx -= bias_x_; gy -= bias_y_; gz -= bias_z_;
    return true;
}

bool IMU_GY521::calibrateGyro(uint16_t samples) {
    if (samples == 0) samples = 1;
    bias_x_ = bias_y_ = bias_z_ = 0.0f;
    float sx = 0, sy = 0, sz = 0, ax, ay, az, gx, gy, gz;
    for (uint16_t i = 0; i < samples; i++) {
        if (!readRaw(ax, ay, az, gx, gy, gz)) return false;
        sx += gx; sy += gy; sz += gz;
        HAL_Delay(5);
    }
    sx /= samples; sy /= samples; sz /= samples;
    if (std::fabs(sx) > 10.0f || std::fabs(sy) > 10.0f || std::fabs(sz) > 10.0f) return false;  // arm was moving
    bias_x_ = sx; bias_y_ = sy; bias_z_ = sz;
    return true;
}
