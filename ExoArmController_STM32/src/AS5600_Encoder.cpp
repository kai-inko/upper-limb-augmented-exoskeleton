#include "AS5600_Encoder.h"

#define AS5600_REG_CONF_H   0x07     // CONF[13:8]
#define AS5600_REG_CONF_L   0x08     // CONF[7:0]
#define AS5600_REG_STATUS   0x0B     // MH (bit3), ML (bit4), MD (bit5)
#define AS5600_REG_RAWANGLE 0x0C     // 0x0C/0x0D = RAW ANGLE, 0x0E/0x0F = ANGLE (filtered)  -> contiguous with STATUS
#define AS5600_REG_AGC      0x1A

#define AS5600_I2C_TIMEOUT_MS  10

AS5600_Encoder::AS5600_Encoder(I2C_HandleTypeDef* hi2c, uint16_t addr)
    : hi2c_(hi2c), dev_addr_(addr), status_(0), errors_(0) {}

bool AS5600_Encoder::writeReg(uint8_t reg, uint8_t val) {
    return HAL_I2C_Mem_Write(hi2c_, dev_addr_, reg, I2C_MEMADD_SIZE_8BIT, &val, 1, AS5600_I2C_TIMEOUT_MS) == HAL_OK;
}

bool AS5600_Encoder::readBlock(uint8_t reg, uint8_t* buf, uint16_t len) {
    return HAL_I2C_Mem_Read(hi2c_, dev_addr_, reg, I2C_MEMADD_SIZE_8BIT, buf, len, AS5600_I2C_TIMEOUT_MS) == HAL_OK;
}

bool AS5600_Encoder::begin() {
    uint8_t st = 0;
    if (!readBlock(AS5600_REG_STATUS, &st, 1)) return false;      // no ACK: wiring / address / pull-ups
    status_ = st;

    // CONF: power mode normal, hysteresis off, analog output, PWM n/a, watchdog off,
    //       slow filter = 2x (fastest response, ~0.3 ms), fast-filter threshold = slow filter only.
    bool ok = true;
    ok &= writeReg(AS5600_REG_CONF_H, 0x03);                      // SF = 0b11 (2x)
    ok &= writeReg(AS5600_REG_CONF_L, 0x00);
    if (!ok) return false;
    HAL_Delay(2);

    if (!readBlock(AS5600_REG_STATUS, &st, 1)) return false;
    status_ = st;
    return magnetDetected();                                      // MD = 1 required
}

bool AS5600_Encoder::readAngleRaw(uint16_t& raw12) {
    uint8_t d[5];
    if (!readBlock(AS5600_REG_STATUS, d, 5)) {
        if (++errors_ >= 3) {                 // bus lock-up recovery: reset the I2C peripheral
            HAL_I2C_DeInit(hi2c_);
            HAL_I2C_Init(hi2c_);
            errors_ = 0;
        }
        return false;
    }
    errors_ = 0;
    status_ = d[0] & 0x38;
    if (!magnetDetected()) return false;      // magnet missing / fell off -> never report a stale angle
    raw12 = (uint16_t)(((d[3] & 0x0F) << 8) | d[4]);              // ANGLE register (filtered, = RAW with ZPOS/MPOS unset)
    return true;
}

bool AS5600_Encoder::readAngleDeg(float& angle_deg) {
    uint16_t raw;
    if (!readAngleRaw(raw)) return false;
    angle_deg = (float)raw * (360.0f / 4096.0f);
    return true;
}

bool AS5600_Encoder::readAgc(uint8_t& agc) {
    return readBlock(AS5600_REG_AGC, &agc, 1);
}
