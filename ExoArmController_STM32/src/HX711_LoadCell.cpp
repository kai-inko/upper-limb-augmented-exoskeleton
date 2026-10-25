#include "HX711_LoadCell.h"

HX711_LoadCell::HX711_LoadCell(GPIO_TypeDef* dout_port, uint16_t dout_pin, 
                               GPIO_TypeDef* sck_port, uint16_t sck_pin)
    : dout_port_(dout_port), dout_pin_(dout_pin), 
      sck_port_(sck_port), sck_pin_(sck_pin), 
      offset_(0), scale_(1.0f), gain_pulses_(25) {}

void HX711_LoadCell::begin() {
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // Configure DOUT Pin as Input
    GPIO_InitStruct.Pin = dout_pin_;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(dout_port_, &GPIO_InitStruct);

    // Configure SCK Pin as Output Push-Pull
    GPIO_InitStruct.Pin = sck_pin_;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(sck_port_, &GPIO_InitStruct);

    HAL_GPIO_WritePin(sck_port_, sck_pin_, GPIO_PIN_RESET);
}

bool HX711_LoadCell::isReady(uint32_t timeout_ms) {
    uint32_t start = HAL_GetTick();
    while (HAL_GPIO_ReadPin(dout_port_, dout_pin_) == GPIO_PIN_SET) {
        if ((HAL_GetTick() - start) >= timeout_ms) {
            return false;
        }
    }
    return true;
}

int32_t HX711_LoadCell::readRaw() {
    if (!isReady(100)) return 0;

    uint32_t count = 0;

    for (int i = 0; i < 24; i++) {
        HAL_GPIO_WritePin(sck_port_, sck_pin_, GPIO_PIN_SET);
        shortDelay();
        count = count << 1;
        HAL_GPIO_WritePin(sck_port_, sck_pin_, GPIO_PIN_RESET);
        shortDelay();
        if (HAL_GPIO_ReadPin(dout_port_, dout_pin_) == GPIO_PIN_SET) {
            count++;
        }
    }

    // Additional gain configuration pulses (1 extra pulse for Gain 128 on Channel A)
    for (int i = 0; i < (gain_pulses_ - 24); i++) {
        HAL_GPIO_WritePin(sck_port_, sck_pin_, GPIO_PIN_SET);
        shortDelay();
        HAL_GPIO_WritePin(sck_port_, sck_pin_, GPIO_PIN_RESET);
        shortDelay();
    }

    // 24-bit sign extension
    if (count & 0x800000) {
        count |= 0xFF000000;
    }

    return static_cast<int32_t>(count);
}

int32_t HX711_LoadCell::readAverage(uint8_t samples) {
    if (samples == 0) samples = 1;
    int64_t sum = 0;
    for (uint8_t i = 0; i < samples; i++) {
        sum += readRaw();
    }
    return static_cast<int32_t>(sum / samples);
}

void HX711_LoadCell::tare(uint8_t samples) {
    int32_t avg = readAverage(samples);
    offset_ = avg;
}

void HX711_LoadCell::setScale(float scale) {
    if (scale != 0.0f) {
        scale_ = scale;
    }
}

float HX711_LoadCell::getScale() const {
    return scale_;
}

float HX711_LoadCell::getUnits(uint8_t samples) {
    int32_t raw_avg = readAverage(samples);
    return static_cast<float>(raw_avg - offset_) / scale_;
}

void HX711_LoadCell::powerDown() {
    HAL_GPIO_WritePin(sck_port_, sck_pin_, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(sck_port_, sck_pin_, GPIO_PIN_SET);
}

void HX711_LoadCell::powerUp() {
    HAL_GPIO_WritePin(sck_port_, sck_pin_, GPIO_PIN_RESET);
}

inline void HX711_LoadCell::shortDelay() {
    for (volatile int i = 0; i < 5; i++) {
        __NOP();
    }
}