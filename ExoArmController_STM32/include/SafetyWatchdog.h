#ifndef SAFETY_WATCHDOG_H
#define SAFETY_WATCHDOG_H

#include "stm32f4xx_hal.h"

// Data-freshness monitor. kick() ONLY when a VALID sample arrived; expired() tells the
// supervisor the sensor went silent. (The independent hardware IWDG guards the MCU itself.)
class SafetyWatchdog {
public:
    explicit SafetyWatchdog(uint32_t timeout_ms) : timeout_ms_(timeout_ms), last_kick_(0) {}
    void kick()  { last_kick_ = HAL_GetTick(); }
    bool expired() const { return (HAL_GetTick() - last_kick_) > timeout_ms_; }
private:
    uint32_t timeout_ms_;
    uint32_t last_kick_;
};

#endif // SAFETY_WATCHDOG_H
