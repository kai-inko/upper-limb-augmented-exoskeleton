#ifndef AS5600_ENCODER_H
#define AS5600_ENCODER_H

#include "stm32f4xx_hal.h"
#include <cstdint>

// AS5600 12-bit contactless magnetic rotary encoder (I2C, fixed 7-bit address 0x36).
// A diametrically magnetised magnet must sit on the elbow axis, ~0.5-3 mm above the chip.
class AS5600_Encoder {
public:
    AS5600_Encoder(I2C_HandleTypeDef* hi2c, uint16_t addr);

    // Probe the chip, configure the output filter for low latency, and require a detected magnet.
    bool begin();

    // One burst read (STATUS + RAW + ANGLE). Returns false on an I2C error OR when no magnet is detected.
    // angle_deg is the 0..360 sensor-frame angle (12-bit, 0.088 deg/LSB). Auto re-initialises the I2C
    // peripheral after 3 consecutive bus errors.
    bool readAngleDeg(float& angle_deg);
    bool readAngleRaw(uint16_t& raw12);

    bool readAgc(uint8_t& agc);                 // automatic gain control, 0..255 (mid-range = good magnet distance)

    // Magnet flags from the most recent successful read.
    bool magnetDetected()  const { return (status_ & 0x20) != 0; }
    bool magnetTooWeak()   const { return (status_ & 0x10) != 0; }
    bool magnetTooStrong() const { return (status_ & 0x08) != 0; }
    uint8_t status() const { return status_; }

private:
    I2C_HandleTypeDef* hi2c_;
    uint16_t dev_addr_;
    uint8_t  status_;
    uint8_t  errors_;

    bool writeReg(uint8_t reg, uint8_t val);
    bool readBlock(uint8_t reg, uint8_t* buf, uint16_t len);
};

#endif // AS5600_ENCODER_H
