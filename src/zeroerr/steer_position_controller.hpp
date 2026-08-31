// Closes a steering position loop on the host over a CSV (velocity-mode)
// actuator -- see zeroerr_actuator.hpp's class comment for why: Profile
// Position mode's on-drive trajectory generator had a real-hardware
// failure mode (cleanly acknowledged retargets, zero corrective torque,
// position frozen for seconds) that resisted diagnosis after ruling out
// hardware, wiring, Control Source, and EtherCAT frame/sync health.
//
// This is a plain P controller on wrapped angle error, accel-limited on
// its own output -- deliberately simple and fully visible, in contrast to
// the opaque on-drive profiler it replaces. Pure P control means a
// steady-state tracking lag proportional to how fast the desired angle is
// changing (lag = velocity / kp_rad_s); acceptable for swerve steering,
// which spends most of its time holding or slowly retargeting rather than
// sweeping continuously at high angular rates.
#pragma once

#include <algorithm>
#include <cmath>

namespace zeroerr {

// A reasonable starting point (rad/s of commanded rate per radian of
// angle error, i.e. a ~150ms time constant) -- tune on the bench via
// whichever tool's --steer-kp flag exposes this, not by editing this
// default in place.
constexpr double kDefaultSteerKp = 6.0;

class SteerPositionController {
public:
    SteerPositionController(double max_rate_rad_s, double max_accel_rad_s2, double kp = kDefaultSteerKp)
        : max_rate_rad_s_(max_rate_rad_s), max_accel_rad_s2_(max_accel_rad_s2), kp_(kp) {}

    // desired_angle_rad/actual_angle_rad are both wheel-frame radians in
    // any mutually-consistent representation -- the error is wrapped to
    // (-pi, pi] via std::remainder, so neither needs to be pre-normalized
    // or unwrapped. Returns the next commanded angular velocity (rad/s),
    // accel-limited relative to the value returned by the previous call.
    double update(double desired_angle_rad, double actual_angle_rad, double dt_s) {
        const double error_rad = std::remainder(desired_angle_rad - actual_angle_rad, 2.0 * M_PI);
        const double target_velocity_rad_s = std::clamp(kp_ * error_rad, -max_rate_rad_s_, max_rate_rad_s_);
        const double max_step = max_accel_rad_s2_ * dt_s;
        last_velocity_rad_s_ += std::clamp(target_velocity_rad_s - last_velocity_rad_s_, -max_step, max_step);
        return last_velocity_rad_s_;
    }

    // Zeros the accel-limiter's memory -- call when a module is gated off
    // (not actively being commanded) so its next real command doesn't
    // inherit a stale velocity from before the gap.
    void reset() { last_velocity_rad_s_ = 0.0; }

    [[nodiscard]] double last_velocity_rad_s() const { return last_velocity_rad_s_; }

private:
    double max_rate_rad_s_;
    double max_accel_rad_s2_;
    double kp_;
    double last_velocity_rad_s_ = 0.0;
};

}  // namespace zeroerr
