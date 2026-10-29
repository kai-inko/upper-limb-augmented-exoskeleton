#ifndef FILTERS_H
#define FILTERS_H

#include <cmath>
#include <cstdint>

class LowPassFilter {
public:
    LowPassFilter(float cutoff_hz, float sample_time) : alpha_(1.0f), y_(0.0f) {
        if (cutoff_hz > 0.0f) {
            float rc = 1.0f / (2.0f * 3.14159265f * cutoff_hz);
            alpha_ = sample_time / (rc + sample_time);
        }
    }
    float update(float x) { y_ += alpha_ * (x - y_); return y_; }
    void  reset(float v = 0.0f) { y_ = v; }
    float value() const { return y_; }
private:
    float alpha_, y_;
};

// Fuses gyro rate (deg/s) with an absolute accelerometer angle (deg). Handles +/-180 wrap.
class ComplementaryFilter {
public:
    explicit ComplementaryFilter(float alpha = 0.98f) : alpha_(alpha), angle_(0.0f) {}
    void  reset(float angle_deg) { angle_ = angle_deg; }
    float update(float gyro_dps, float accel_angle_deg, bool accel_valid, float dt) {
        angle_ += gyro_dps * dt;
        if (accel_valid) {
            float err = accel_angle_deg - angle_;
            while (err >  180.0f) err -= 360.0f;
            while (err < -180.0f) err += 360.0f;
            angle_ += (1.0f - alpha_) * err;
        }
        while (angle_ >  180.0f) angle_ -= 360.0f;
        while (angle_ < -180.0f) angle_ += 360.0f;
        return angle_;
    }
    float value() const { return angle_; }
private:
    float alpha_, angle_;
};

// Limits how fast a command may change per call (protects gears / arm from jerks).
class SlewLimiter {
public:
    explicit SlewLimiter(float max_step) : step_(max_step), y_(0.0f) {}
    float update(float target) {
        float d = target - y_;
        if (d >  step_) d =  step_;
        if (d < -step_) d = -step_;
        y_ += d;
        return y_;
    }
    void  reset(float v = 0.0f) { y_ = v; }
    float value() const { return y_; }
private:
    float step_, y_;
};

inline float applyDeadband(float x, float band) {
    if (x >  band) return x - band;
    if (x < -band) return x + band;
    return 0.0f;
}

inline float clampf(float x, float lo, float hi) { return x < lo ? lo : (x > hi ? hi : x); }

// Wraps an angle into (-180, +180].
inline float wrap180(float a) {
    while (a >  180.0f) a -= 360.0f;
    while (a <= -180.0f) a += 360.0f;
    return a;
}

// Limb inclination from gravity only (MPU-6050 on the UPPER ARM).
// 0 deg = hanging down (ax = -1 g), 90 deg = horizontal forward (ay = +1 g), 180 deg = pointing up.
inline float limbAngleFromAccel(float ax, float ay) {
    return std::atan2(ay, -ax) * 57.2957795f;
}
inline bool accelMagnitudeValid(float ax, float ay, float az) {
    float m2 = ax * ax + ay * ay + az * az;     // 0.85 g .. 1.15 g
    return (m2 > 0.7225f) && (m2 < 1.3225f);
}

#endif // FILTERS_H
