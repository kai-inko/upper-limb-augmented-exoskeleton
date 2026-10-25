#ifndef HX711_LOADCELL_H
#define HX711_LOADCELL_H

#include "stm32f1xx_hal.h"
#include <cstdint>

class HX711_LoadCell {
public:
    HX711_LoadCell(GPIO_TypeDef* dout_port, uint16_t dout_pin, 
                   GPIO_TypeDef* sck_port, uint16_t sck_pin);

    void begin();
    bool isReady(uint32_t timeout_ms = 100);
    int32_t readRaw();
    int32_t readAverage(uint8_t samples = 10);
    
    void tare(uint8_t samples = 10);
    void setScale(float scale);
    float getScale() const;
    
    float getUnits(uint8_t samples = 1);
    
    void powerDown();
    void powerUp();

private:
    GPIO_TypeDef* dout_port_;
    uint16_t dout_pin_;
    GPIO_TypeDef* sck_port_;
    uint16_t sck_pin_;

    int32_t offset_;
    float scale_;
    uint8_t gain_pulses_; // 25 = Gain 128 (Channel A), 26 = Gain 32 (Channel B), 27 = Gain 64 (Channel A)

    void shortDelay();
};

#endif // HX711_LOADCELL_H