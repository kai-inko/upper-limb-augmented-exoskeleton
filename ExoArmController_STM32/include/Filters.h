#ifndef FILTERS_H
#define FILTERS_H

#include <Arduino.h>

// Exponential Moving Average (Low Pass) Filter
class LowPassFilter {
public:
    LowPassFilter(float alpha, float initialVal = 0.0f)
        : alpha(alpha), filteredValue(initialVal), initialized(false) {}

    void setAlpha(float newAlpha) {
        alpha = constrain(newAlpha, 0.01f, 1.0f);
    }

    float update(float input) {
        if (!initialized) {
            filteredValue = input;
            initialized = true;
        } else {
            filteredValue = (alpha * input) + ((1.0f - alpha) * filteredValue);
        }
        return filteredValue;
    }

    float getValue() const { return filteredValue; }
    void reset(float val = 0.0f) { filteredValue = val; initialized = false; }

private:
    float alpha; // Smoothing factor: closer to 0 = heavier smoothing, closer to 1 = faster response
    float filteredValue;
    bool  initialized;
};

// Slew Rate Limiter (limits maximum rate of change per cycle)
class SlewRateLimiter {
public:
    SlewRateLimiter(float maxStepPerCycle) : maxStep(maxStepPerCycle), currentVal(0.0f) {}

    void setMaxStep(float step) { maxStep = step; }

    float update(float target) {
        float delta = target - currentVal;
        if (delta > maxStep) {
            currentVal += maxStep;
        } else if (delta < -maxStep) {
            currentVal -= maxStep;
        } else {
            currentVal = target;
        }
        return currentVal;
    }

    float getValue() const { return currentVal; }
    void reset(float val = 0.0f) { currentVal = val; }

private:
    float maxStep;
    float currentVal;
};

// Apply symmetric deadband
inline float applyDeadband(float value, float threshold) {
    if (abs(value) < threshold) {
        return 0.0f;
    }
    return value;
}

#endif // FILTERS_H
