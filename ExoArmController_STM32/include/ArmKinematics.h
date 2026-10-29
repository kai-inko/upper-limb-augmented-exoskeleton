#ifndef ARM_KINEMATICS_H
#define ARM_KINEMATICS_H

#include <cmath>
#include "Filters.h"

// ---------------------------------------------------------------------------------------------
// Elbow joint estimator: fuses the ABSOLUTE AS5600 angle with the fast RELATIVE optical encoder.
//
//   * Every tick the optical delta is added (dead-reckoning: fine resolution, no latency).
//   * When a valid AS5600 sample is available, the estimate is pulled toward it by `fusion_gain`
//     (removes drift, lost counts and gives the optical encoder its absolute zero).
//   * mismatch() = AS5600 - (optical-predicted angle) BEFORE the correction. A large persistent value
//     means one of the two sensors is lying (slipped magnet, lost counts, loose encoder coupling).
//   * velocity() is the optical-encoder speed, low-pass filtered (deg/s, + = flexion).
// All angles are in degrees, 0 = elbow fully extended, + = flexion.
// ---------------------------------------------------------------------------------------------
class JointEstimator {
public:
    JointEstimator(float fusion_gain, float vel_cutoff_hz, float dt)
        : k_(fusion_gain), dt_(dt), angle_(0.0f), mismatch_(0.0f), vel_lpf_(vel_cutoff_hz, dt) {}

    // Home the estimate to an absolute angle (AS5600 at boot).
    void reset(float abs_deg) { angle_ = wrap180(abs_deg); mismatch_ = 0.0f; vel_lpf_.reset(0.0f); }

    void update(float opt_delta_deg, float abs_deg, bool abs_valid) {
        angle_ = wrap180(angle_ + opt_delta_deg);
        if (abs_valid) {
            mismatch_ = wrap180(abs_deg - angle_);
            angle_    = wrap180(angle_ + k_ * mismatch_);
        }
        vel_lpf_.update(opt_delta_deg / dt_);
    }

    float angle()    const { return angle_; }
    float velocity() const { return vel_lpf_.value(); }
    float mismatch() const { return mismatch_; }

private:
    float k_, dt_, angle_, mismatch_;
    LowPassFilter vel_lpf_;
};

// Forearm placement in the sagittal plane (shoulder = origin, x forward, y up), metres.
//   ua_deg   : upper-arm inclination from gravity (0 = hanging down, 90 = forward horizontal)
//   fore_deg : forearm inclination from gravity   (= ua_deg + elbow flexion)
// Returns the elbow position and the wrist/cuff position.
struct ArmPlacement { float elbow_x, elbow_y, wrist_x, wrist_y; };

inline ArmPlacement forearmPlacement(float ua_deg, float fore_deg, float upper_len_m, float fore_len_m) {
    const float d2r = 0.0174532925f;
    ArmPlacement p;
    p.elbow_x =  upper_len_m * std::sin(ua_deg * d2r);
    p.elbow_y = -upper_len_m * std::cos(ua_deg * d2r);
    p.wrist_x = p.elbow_x +  fore_len_m * std::sin(fore_deg * d2r);
    p.wrist_y = p.elbow_y -  fore_len_m * std::cos(fore_deg * d2r);
    return p;
}

#endif // ARM_KINEMATICS_H
