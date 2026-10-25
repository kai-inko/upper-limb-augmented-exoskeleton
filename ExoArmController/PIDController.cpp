#include "PIDController.h"

PIDController::PIDController(float kp, float ki, float kd, float outMin, float outMax)
    : Kp(kp), Ki(ki), Kd(kd),
      outMin(outMin), outMax(outMax),
      iLimitMin(outMin * 0.7f), iLimitMax(outMax * 0.7f),
      integralSum(0.0f), lastMeasurement(0.0f), lastDerivative(0.0f),
      isFirstRun(true), pTerm(0.0f), iTerm(0.0f), dTerm(0.0f) {}

void PIDController::setTunings(float kp, float ki, float kd) {
    Kp = kp;
    Ki = ki;
    Kd = kd;
}

void PIDController::setOutputLimits(float minVal, float maxVal) {
    outMin = minVal;
    outMax = maxVal;
}

void PIDController::setIntegralLimits(float minVal, float maxVal) {
    iLimitMin = minVal;
    iLimitMax = maxVal;
}

void PIDController::reset() {
    integralSum = 0.0f;
    lastDerivative = 0.0f;
    isFirstRun = true;
    pTerm = 0.0f;
    iTerm = 0.0f;
    dTerm = 0.0f;
}

float PIDController::compute(float setpoint, float measurement, float dt_seconds) {
    if (dt_seconds <= 0.0f) {
        dt_seconds = 0.02f; // Fallback to 50Hz if timing anomaly occurs
    }

    float error = setpoint - measurement;

    // Proportional Term
    pTerm = Kp * error;

    // Integral Term with Anti-Windup Clamping
    integralSum += error * dt_seconds;
    if (integralSum > iLimitMax) integralSum = iLimitMax;
    if (integralSum < iLimitMin) integralSum = iLimitMin;
    iTerm = Ki * integralSum;

    // Derivative Term on Measurement (eliminates derivative kick on setpoint step changes)
    if (isFirstRun) {
        lastMeasurement = measurement;
        lastDerivative = 0.0f;
        isFirstRun = false;
    }

    float rawDerivative = -(measurement - lastMeasurement) / dt_seconds;
    lastMeasurement = measurement;

    // 1st-order low-pass filter on derivative (cutoff ~ 10Hz) to prevent high-frequency noise amplification
    const float alpha = 0.7f;
    float filteredDerivative = alpha * rawDerivative + (1.0f - alpha) * lastDerivative;
    lastDerivative = filteredDerivative;
    dTerm = Kd * filteredDerivative;

    // Sum and constrain to actuator saturation limits
    float output = pTerm + iTerm + dTerm;
    if (output > outMax) output = outMax;
    if (output < outMin) output = outMin;

    return output;
}
