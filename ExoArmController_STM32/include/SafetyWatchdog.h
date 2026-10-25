#ifndef SAFETY_WATCHDOG_H
#define SAFETY_WATCHDOG_H

#include <Arduino.h>
#include "Config.h"

enum SafetyState {
    SAFETY_OK = 0,
    SAFETY_ESTOP_TRIGGERED,
    SAFETY_MPU_HARDWARE_INT_TRIPPED,
    SAFETY_IMU_SENSOR_FAULT,
    SAFETY_LOAD_CELL_OVERLOAD,
    SAFETY_ANGLE_LIMIT_MIN,
    SAFETY_ANGLE_LIMIT_MAX,
    SAFETY_MOTOR_STALL_DETECTED
};

class SafetyWatchdog {
public:
    SafetyWatchdog()
        : currentState(SAFETY_OK),
          stallTimerStart(0),
          lastAngleForStallCheck(0.0f),
          isMonitoringStall(false),
          hardwareIntFired(false) {}

    void init() {
        pinMode(PIN_ESTOP, INPUT_PULLUP);
        pinMode(PIN_MPU_INT, INPUT);
    }

    // Called from hardware EXTI ISR when MPU6050 INT pin fires
    void triggerHardwareInterruptFromISR(float currentAngle) {
        // If angle reaches beyond safe degree range, trip immediate emergency lockdown.
        // ISR rules: keep it to flag writes + one GPIO write. analogWrite() reconfigures
        // TIM2 and is not safe to call from an ISR while the main loop may be inside it.
        // Dropping the BTS7960 enable pin de-energizes both half-bridges immediately; the
        // main loop sees the flag on its next pass and zeroes the PWM duty (emergencyShutdown).
        if (currentAngle <= MPU_INT_MIN_DEG || currentAngle >= MPU_INT_MAX_DEG) {
            currentState = SAFETY_MPU_HARDWARE_INT_TRIPPED;
            hardwareIntFired = true;
            digitalWrite(PIN_BTS_EN, LOW);
        }
    }

    // Comprehensive safety evaluation
    SafetyState checkSafety(bool imuHealthy, float currentAngleDeg, float weightKg, float commandedPWM) {
        // 1. Hardware E-Stop switch check (PA6)
        if (digitalRead(PIN_ESTOP) == LOW) {
            currentState = SAFETY_ESTOP_TRIGGERED;
            return currentState;
        }

        // 2. Hardware MPU INT Flag Check
        if (hardwareIntFired) {
            currentState = SAFETY_MPU_HARDWARE_INT_TRIPPED;
            return currentState;
        }

        // 3. IMU Communication integrity
        if (!imuHealthy) {
            currentState = SAFETY_IMU_SENSOR_FAULT;
            return currentState;
        }

        // 4. Load cell structural overload check in kg
        if (abs(weightKg) > (LOAD_CELL_MAX_KG * 0.92f)) {
            currentState = SAFETY_LOAD_CELL_OVERLOAD;
            return currentState;
        }

        // 5. Joint Angle Range Interlock
        if (currentAngleDeg <= ANGLE_MIN_DEG) {
            if (commandedPWM < 0) {
                currentState = SAFETY_ANGLE_LIMIT_MIN;
                return currentState;
            }
        } else if (currentAngleDeg >= ANGLE_MAX_DEG) {
            if (commandedPWM > 0) {
                currentState = SAFETY_ANGLE_LIMIT_MAX;
                return currentState;
            }
        }

        // 6. Motor stall detection
        if (abs(commandedPWM) > 170) {
            if (!isMonitoringStall) {
                isMonitoringStall = true;
                stallTimerStart = millis();
                lastAngleForStallCheck = currentAngleDeg;
            } else {
                if (millis() - stallTimerStart > 2200) {
                    if (abs(currentAngleDeg - lastAngleForStallCheck) < 1.8f) {
                        currentState = SAFETY_MOTOR_STALL_DETECTED;
                        return currentState;
                    }
                    stallTimerStart = millis();
                    lastAngleForStallCheck = currentAngleDeg;
                }
            }
        } else {
            isMonitoringStall = false;
        }

        currentState = SAFETY_OK;
        return currentState;
    }

    // Boundary speed governor: linearly tapers down motor PWM as the joint nears limit
    float governOutputNearLimits(float targetPWM, float currentAngleDeg) {
        if (targetPWM < 0 && (currentAngleDeg < ANGLE_MIN_DEG + ANGLE_SAFETY_MARGIN)) {
            float dist = currentAngleDeg - ANGLE_MIN_DEG;
            if (dist <= 0) return 0.0f;
            return targetPWM * (dist / ANGLE_SAFETY_MARGIN);
        }

        if (targetPWM > 0 && (currentAngleDeg > ANGLE_MAX_DEG - ANGLE_SAFETY_MARGIN)) {
            float dist = ANGLE_MAX_DEG - currentAngleDeg;
            if (dist <= 0) return 0.0f;
            return targetPWM * (dist / ANGLE_SAFETY_MARGIN);
        }

        return targetPWM;
    }

    const char* getStateDescription() const {
        switch (currentState) {
            case SAFETY_OK: return "OK";
            case SAFETY_ESTOP_TRIGGERED: return "EMERGENCY STOP (PA6) PRESSED";
            case SAFETY_MPU_HARDWARE_INT_TRIPPED: return "MPU6050 HARDWARE INT: DEGREE RANGE TRIPPED";
            case SAFETY_IMU_SENSOR_FAULT: return "IMU I2C BUS DISCONNECTED";
            case SAFETY_LOAD_CELL_OVERLOAD: return "LOAD CELL 20KG OVERLOAD (>18.5kg)";
            case SAFETY_ANGLE_LIMIT_MIN: return "LOWER JOINT LIMIT (EXTENSION)";
            case SAFETY_ANGLE_LIMIT_MAX: return "UPPER JOINT LIMIT (FLEXION)";
            case SAFETY_MOTOR_STALL_DETECTED: return "MOTOR STALL DETECTED";
            default: return "UNKNOWN FAULT";
        }
    }

    void resetFault() {
        currentState = SAFETY_OK;
        hardwareIntFired = false;
        isMonitoringStall = false;
    }

    SafetyState getState() const { return currentState; }

private:
    volatile SafetyState currentState;   // written from the EXTI ISR and the main loop
    unsigned long stallTimerStart;
    float lastAngleForStallCheck;
    bool isMonitoringStall;
    volatile bool hardwareIntFired;
};

#endif // SAFETY_WATCHDOG_H
