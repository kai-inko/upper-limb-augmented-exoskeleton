#include "PIDController.h"

PIDController::PIDController(float kp, float ki, float kd, float out_min, float out_max)
    : kp_(kp), ki_(ki), kd_(kd), out_min_(out_min), out_max_(out_max), integral_(0.0f), prev_error_(0.0f) {}

float PIDController::compute(float setpoint, float measured, float dt) {
    if (dt <= 0.0f) return 0.0f;

    float error = setpoint - measured;
    integral_ += error * dt;
    
    // Anti-windup clamping
    integral_ = std::clamp(integral_, out_min_, out_max_);

    float derivative = (error - prev_error_) / dt;
    float output = (kp_ * error) + (ki_ * integral_) + (kd_ * derivative);

    prev_error_ = error;
    return std::clamp(output, out_min_, out_max_);
}

void PIDController::reset() {
    integral_ = 0.0f;
    prev_error_ = 0.0f;
}