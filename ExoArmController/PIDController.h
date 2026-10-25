#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include <Arduino.h>

class PIDController {
public:
    PIDController(float kp, float ki, float kd, float outMin, float outMax);

    // Compute PID output given setpoint and current measurement
    float compute(float setpoint, float measurement, float dt_seconds);

    // Dynamic gain adjustment
    void setTunings(float kp, float ki, float kd);
    void setOutputLimits(float minVal, float maxVal);
    void setIntegralLimits(float minVal, float maxVal);
    
    // Reset state (clear integrator and derivative memory)
    void reset();

    // Get internal state for diagnostics
    float getPterm() const { return pTerm; }
    float getIterm() const { return iTerm; }
    float getDterm() const { return dTerm; }
    float getIntegralSum() const { return integralSum; }

private:
    float Kp;
    float Ki;
    float Kd;

    float outMin;
    float outMax;
    float iLimitMin;
    float iLimitMax;

    float integralSum;
    float lastMeasurement;
    float lastDerivative;
    bool  isFirstRun;

    // Diagnostic logging terms
    float pTerm;
    float iTerm;
    float dTerm;
};

#endif // PID_CONTROLLER_H
