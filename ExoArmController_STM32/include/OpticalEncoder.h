#ifndef OPTICAL_ENCODER_H
#define OPTICAL_ENCODER_H

#include "stm32f4xx_hal.h"
#include <cstdint>

// 1000 PPR optical incremental encoder read through TIM3 in hardware quadrature mode (X4 = 4000 counts/rev).
// The counter free-runs in hardware (see Board_Init); this class only turns it into wrap-safe deltas.
// The encoder is RELATIVE: it has no absolute zero, so the joint estimator homes it to the AS5600 at boot.
class OpticalEncoder {
public:
    explicit OpticalEncoder(TIM_HandleTypeDef* htim) : htim_(htim), last_(0), total_(0) {}

    // Latch the current hardware count as the reference (call once after Board_Init).
    void begin() { last_ = (uint16_t)__HAL_TIM_GET_COUNTER(htim_); total_ = 0; }

    // Signed counts since the previous call. Correct as long as fewer than 32768 counts accrue between calls
    // (that is > 8 encoder revolutions per 10 ms tick).
    int32_t readDeltaCounts() {
        uint16_t now = (uint16_t)__HAL_TIM_GET_COUNTER(htim_);
        int16_t  d   = (int16_t)(uint16_t)(now - last_);
        last_  = now;
        total_ += d;
        return d;
    }

    int32_t totalCounts() const { return total_; }
    void    resetTotal()        { total_ = 0; }

private:
    TIM_HandleTypeDef* htim_;
    uint16_t last_;
    int32_t  total_;
};

#endif // OPTICAL_ENCODER_H
