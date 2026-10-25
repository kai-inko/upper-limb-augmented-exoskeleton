#ifndef SAFETY_WATCHDOG_H
#define SAFETY_WATCHDOG_H

#include <Arduino.h>
#include "Config.h"

enum SafetyState {
    SAFETY_OK = 0,
    SAFETY_ESTOP_TRIGGERED,
    SAFETY_IMU_SENSOR_FAULT,
    SAFETY_POT_SENSOR_FAULT,
    SAFETY_ANGLE_LIMIT_MIN,
    SAFETY_ANGLE_LIMIT_MAX,
    SAFETY_MOTOR_STALL_DETECTED,
    SAFETY_LOAD_CELL_OVERLOAD
};

class SafetyWatchdog {
public:
    SafetyWatchdog()
        : currentState(SAFETY_OK),
          stallTimerStart(0),
          lastAngleForStallCheck(0.0f),
          isMonitoringStall(false) {}

    void init() {
        pinMode(PIN_ESTOP, INPUT_PULLUP);
    }

    // Evaluate hardware integrity, IMU health, load limits, and angle limits
    SafetyState checkSafety(bool imuHealthy, float currentAngleDeg, float interactionForceN, float commandedPWM) {
        // 1. Hardware E-Stop pin check
        if (digitalRead(PIN_ESTOP) == LOW) {
            currentState = SAFETY_ESTOP_TRIGGERED;
            return currentState;
        }

        // 2. IMU Sensor Integrity Check (GY-521 I2C communication)
        if (!imuHealthy) {
            currentState = SAFETY_IMU_SENSOR_FAULT;
            return currentState;
        }

        // 3. 20kg Load Cell Structural Overload Protection (> 180 N = approx 18.4 kg)
        if (abs(interactionForceN) > (LOAD_CELL_MAX_KG * 9.80665f * 0.92f)) {
            currentState = SAFETY_LOAD_CELL_OVERLOAD;
            return currentState;
        }

        // 4. Physical angle boundary check
        if (currentAngleDeg <= ANGLE_MIN_DEG) {
            if (commandedPWM < 0) { // Command is trying to extend further downwards
                currentState = SAFETY_ANGLE_LIMIT_MIN;
                return currentState;
            }
        } else if (currentAngleDeg >= ANGLE_MAX_DEG) {
            if (commandedPWM > 0) { // Command is trying to flex further upwards
                currentState = SAFETY_ANGLE_LIMIT_MAX;
                return currentState;
            }
        }

        // 5. Motor Stall Detection: Detect if motor is receiving high power but arm cannot move
        if (abs(commandedPWM) > 170) {
            if (!isMonitoringStall) {
                isMonitoringStall = true;
                stallTimerStart = millis();
                lastAngleForStallCheck = currentAngleDeg;
            } else {
                if (millis() - stallTimerStart > 2200) { // 2.2 seconds stall threshold
                    if (abs(currentAngleDeg - lastAngleForStallCheck) < 1.8f) {
                        currentState = SAFETY_MOTOR_STALL_DETECTED;
                        return currentState;
                    }
                    // Reset stall benchmark if motion is detected
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

    // Dynamic speed governor near boundaries: decelerates motor smoothly when nearing mechanical limit
    float governOutputNearLimits(float targetPWM, float currentAngleDeg) {
        // Near lower limit (extension)
        if (targetPWM < 0 && (currentAngleDeg < ANGLE_MIN_DEG + ANGLE_SAFETY_MARGIN)) {
            float dist = currentAngleDeg - ANGLE_MIN_DEG;
            if (dist <= 0) return 0.0f; // Complete block
            float scale = dist / ANGLE_SAFETY_MARGIN; // Decelerate linearly to zero
            return targetPWM * scale;
        }

        // Near upper limit (flexion)
        if (targetPWM > 0 && (currentAngleDeg > ANGLE_MAX_DEG - ANGLE_SAFETY_MARGIN)) {
            float dist = ANGLE_MAX_DEG - currentAngleDeg;
            if (dist <= 0) return 0.0f; // Complete block
            float scale = dist / ANGLE_SAFETY_MARGIN;
            return targetPWM * scale;
        }

        return targetPWM;
    }

    const char* getStateDescription() const {
        switch (currentState) {
            case SAFETY_OK: return "OK";
            case SAFETY_ESTOP_TRIGGERED: return "EMERGENCY STOP PRESSED";
            case SAFETY_IMU_SENSOR_FAULT: return "IMU GY-521 I2C DISCONNECTED / FAULT";
            case SAFETY_POT_SENSOR_FAULT: return "BACKUP POTENTIOMETER FAULT";
            case SAFETY_LOAD_CELL_OVERLOAD: return "LOAD CELL 20KG RATED LIMIT EXCEEDED";
            case SAFETY_ANGLE_LIMIT_MIN: return "LOWER JOINT LIMIT REACHED (EXTENSION)";
            case SAFETY_ANGLE_LIMIT_MAX: return "UPPER JOINT LIMIT REACHED (FLEXION)";
            case SAFETY_MOTOR_STALL_DETECTED: return "MOTOR STALL / OVERLOAD DETECTED";
            default: return "UNKNOWN FAULT";
        }
    }

    void resetFault() {
        currentState = SAFETY_OK;
        isMonitoringStall = false;
    }

    SafetyState getState() const { return currentState; }

private:
    SafetyState currentState;
    unsigned long stallTimerStart;
    float lastAngleForStallCheck;
    bool isMonitoringStall;
};

#endif // SAFETY_WATCHDOG_H
