#include "PIDController.h"

PIDController::PIDController(float kp, float ki, float kd, float out_min, float out_max)
    : kp_(kp), ki_(ki), kd_(kd), out_min_(out_min), out_max_(out_max),
      integral_(0.0f), prev_meas_(0.0f), has_prev_(false) {}

void PIDController::setGains(float kp, float ki, float kd) { kp_ = kp; ki_ = ki; kd_ = kd; }

float PIDController::compute(float setpoint, float measured, float dt) {
    if (dt <= 0.0f) return 0.0f;

    float error = setpoint - measured;
    float p = kp_ * error;

    // Derivative on measurement, skipped on the very first sample (no start-up spike)
    float d = 0.0f;
    if (has_prev_) d = -kd_ * (measured - prev_meas_) / dt;
    prev_meas_ = measured;
    has_prev_  = true;

    // Conditional integration: do not wind up while the output is saturated in the error's direction
    float unsat = p + ki_ * integral_ + d;
    bool sat_hi = (unsat >= out_max_) && (error > 0.0f);
    bool sat_lo = (unsat <= out_min_) && (error < 0.0f);
    if (!sat_hi && !sat_lo) integral_ += error * dt;

    // The integral term itself may never exceed the output range
    if (ki_ > 0.0f) {
        float lim = out_max_ / ki_;
        if (integral_ >  lim) integral_ =  lim;
        if (integral_ < -lim) integral_ = -lim;
    } else {
        integral_ = 0.0f;
    }

    float out = p + ki_ * integral_ + d;
    if (out > out_max_) out = out_max_;
    if (out < out_min_) out = out_min_;
    return out;
}

void PIDController::reset() {
    integral_  = 0.0f;
    prev_meas_ = 0.0f;
    has_prev_  = false;
}
