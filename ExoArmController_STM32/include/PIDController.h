#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include <cstdint>

class PIDController {
public:
    PIDController(float kp, float ki, float kd, float out_min, float out_max);
    // error = setpoint - measured. Derivative acts on the measurement (no kick on setpoint change).
    float compute(float setpoint, float measured, float dt);
    void  reset();
    void  setGains(float kp, float ki, float kd);
private:
    float kp_, ki_, kd_;
    float out_min_, out_max_;
    float integral_;
    float prev_meas_;
    bool  has_prev_;
};

#endif // PID_CONTROLLER_H
