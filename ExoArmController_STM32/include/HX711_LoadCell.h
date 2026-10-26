#ifndef HX711_LOADCELL_H
#define HX711_LOADCELL_H

#include "stm32f1xx_hal.h"
#include <cstdint>

class HX711_LoadCell {
public:
    HX711_LoadCell(GPIO_TypeDef* dout_port, uint16_t dout_pin,
                   GPIO_TypeDef* sck_port,  uint16_t sck_pin);

    void begin();

    // Non-blocking when timeout_ms == 0: returns false immediately if no new conversion is ready.
    // Also returns false (and sets wasSaturated()) when the ADC output is pinned at full scale,
    // which is what a broken bridge / bad wiring looks like.
    bool dataReady() const;
    bool read(int32_t& raw, uint32_t timeout_ms = 0);
    bool readAverage(int32_t& avg, uint8_t samples = 10, uint32_t timeout_ms = 200);
    bool wasSaturated() const { return saturated_; }

    bool  tare(uint8_t samples = 10);
    void  setOffset(int32_t offset) { offset_ = offset; }
    int32_t getOffset() const { return offset_; }
    void  setScale(float counts_per_unit);
    float getScale() const { return scale_; }

    float rawToUnits(int32_t raw) const;
    bool  getUnits(float& units, uint8_t samples = 1, uint32_t timeout_ms = 200);

    void powerDown();
    void powerUp();

private:
    GPIO_TypeDef* dout_port_;
    uint16_t dout_pin_;
    GPIO_TypeDef* sck_port_;
    uint16_t sck_pin_;

    int32_t offset_;
    float   scale_;
    uint8_t gain_pulses_;   // 25 = Gain 128 (Ch A), 26 = Gain 32 (Ch B), 27 = Gain 64 (Ch A)
    bool    saturated_;

    void shortDelay() const;
};

#endif // HX711_LOADCELL_H
