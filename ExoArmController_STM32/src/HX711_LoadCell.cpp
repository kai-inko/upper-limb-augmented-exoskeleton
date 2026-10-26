#include "HX711_LoadCell.h"

HX711_LoadCell::HX711_LoadCell(GPIO_TypeDef* dout_port, uint16_t dout_pin,
                               GPIO_TypeDef* sck_port,  uint16_t sck_pin)
    : dout_port_(dout_port), dout_pin_(dout_pin), sck_port_(sck_port), sck_pin_(sck_pin),
      offset_(0), scale_(1.0f), gain_pulses_(25), saturated_(false) {}

void HX711_LoadCell::begin() {
    GPIO_InitTypeDef g = {0};

    if      (dout_port_ == GPIOA) __HAL_RCC_GPIOA_CLK_ENABLE();
    else if (dout_port_ == GPIOB) __HAL_RCC_GPIOB_CLK_ENABLE();
    else if (dout_port_ == GPIOC) __HAL_RCC_GPIOC_CLK_ENABLE();
    if      (sck_port_ == GPIOA)  __HAL_RCC_GPIOA_CLK_ENABLE();
    else if (sck_port_ == GPIOB)  __HAL_RCC_GPIOB_CLK_ENABLE();
    else if (sck_port_ == GPIOC)  __HAL_RCC_GPIOC_CLK_ENABLE();

    g.Pin = dout_pin_; g.Mode = GPIO_MODE_INPUT; g.Pull = GPIO_PULLUP;     // idle-high line, defined state if unplugged
    HAL_GPIO_Init(dout_port_, &g);

    HAL_GPIO_WritePin(sck_port_, sck_pin_, GPIO_PIN_RESET);
    g.Pin = sck_pin_; g.Mode = GPIO_MODE_OUTPUT_PP; g.Pull = GPIO_NOPULL; g.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(sck_port_, &g);
}

bool HX711_LoadCell::dataReady() const {
    return HAL_GPIO_ReadPin(dout_port_, dout_pin_) == GPIO_PIN_RESET;     // DOUT low = conversion ready
}

bool HX711_LoadCell::read(int32_t& raw, uint32_t timeout_ms) {
    uint32_t start = HAL_GetTick();
    while (!dataReady()) {
        if (timeout_ms == 0 || (HAL_GetTick() - start) >= timeout_ms) return false;
    }

    // SCK must never stay high > 60 us (that powers the chip down), so no interrupt may split the burst.
    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    uint32_t count = 0;
    for (int i = 0; i < 24; i++) {
        HAL_GPIO_WritePin(sck_port_, sck_pin_, GPIO_PIN_SET);
        shortDelay();
        count <<= 1;
        HAL_GPIO_WritePin(sck_port_, sck_pin_, GPIO_PIN_RESET);
        shortDelay();
        if (HAL_GPIO_ReadPin(dout_port_, dout_pin_) == GPIO_PIN_SET) count |= 1u;
    }
    for (int i = 0; i < (int)gain_pulses_ - 24; i++) {          // select gain/channel for the NEXT conversion
        HAL_GPIO_WritePin(sck_port_, sck_pin_, GPIO_PIN_SET);
        shortDelay();
        HAL_GPIO_WritePin(sck_port_, sck_pin_, GPIO_PIN_RESET);
        shortDelay();
    }
    __set_PRIMASK(primask);

    if (count & 0x800000u) count |= 0xFF000000u;                // 24-bit sign extension
    int32_t v = (int32_t)count;

    saturated_ = (v == 0x7FFFFF) || (v == -0x800000);           // pinned at full scale = bridge/wiring fault
    if (saturated_) return false;

    raw = v;
    return true;
}

bool HX711_LoadCell::readAverage(int32_t& avg, uint8_t samples, uint32_t timeout_ms) {
    if (samples == 0) samples = 1;
    int64_t sum = 0;
    for (uint8_t i = 0; i < samples; i++) {
        int32_t r;
        if (!read(r, timeout_ms)) return false;                 // a timeout is an error, not a zero reading
        sum += r;
    }
    avg = (int32_t)(sum / samples);
    return true;
}

bool HX711_LoadCell::tare(uint8_t samples) {
    int32_t avg;
    if (!readAverage(avg, samples, 300)) return false;
    offset_ = avg;
    return true;
}

void HX711_LoadCell::setScale(float counts_per_unit) {
    if (counts_per_unit != 0.0f) scale_ = counts_per_unit;
}

float HX711_LoadCell::rawToUnits(int32_t raw) const {
    return (float)(raw - offset_) / scale_;
}

bool HX711_LoadCell::getUnits(float& units, uint8_t samples, uint32_t timeout_ms) {
    int32_t avg;
    if (!readAverage(avg, samples, timeout_ms)) return false;
    units = rawToUnits(avg);
    return true;
}

void HX711_LoadCell::powerDown() {
    HAL_GPIO_WritePin(sck_port_, sck_pin_, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(sck_port_, sck_pin_, GPIO_PIN_SET);       // held high > 60 us -> power down
}

void HX711_LoadCell::powerUp() {
    HAL_GPIO_WritePin(sck_port_, sck_pin_, GPIO_PIN_RESET);
}

void HX711_LoadCell::shortDelay() const {
    for (volatile int i = 0; i < 5; i++) { __NOP(); }           // ~0.5 us at 72 MHz (spec: 0.2-50 us)
}
