#ifndef SAFETY_WATCHDOG_H
#define SAFETY_WATCHDOG_H

#include "stm32f1xx_hal.h"

class SafetyWatchdog {
public:
    SafetyWatchdog(uint32_t timeout_ms) : timeout_ms_(timeout_ms), last_kick_time_(0), triggered_(false) {}

    void kick() {
        last_kick_time_ = HAL_GetTick();
        triggered_ = false;
    }

    bool checkTimeout() {
        if ((HAL_GetTick() - last_kick_time_) > timeout_ms_) {
            triggered_ = true;
        }
        return triggered_;
    }

    bool isTriggered() const { return triggered_; }

private:
    uint32_t timeout_ms_;
    uint32_t last_kick_time_;
    bool triggered_;
};

#endif // SAFETY_WATCHDOG_H