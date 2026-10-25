#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include <algorithm>
#include <cstdint>

class PIDController {
public:
    PIDController(float kp, float ki, float kd, float out_min, float out_max);
    float compute(float setpoint, float measured, float dt);
    void reset();

private:
    float kp_, ki_, kd_;
    float out_min_, out_max_;
    float integral_;
    float prev_error_;
};

#endif // PID_CONTROLLER_H