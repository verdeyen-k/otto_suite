// 4-wheel swerve kinematics: chassis velocity <-> per-module (wheel speed,
// steering angle). Pure math, no EtherCAT/CiA-402 dependency -- this is
// plan.md's Layer 3, deliberately independent of the driver layer so it's
// testable without any bus/hardware present.
//
// Derived from the same matrix relationship WPILib's SwerveDriveKinematics
// uses (frc-docs "Kinematics and Odometry"; wpilibsuite/allwpilib
// SwerveDriveKinematics.h/.cpp), reimplemented here without pulling in
// WPILib itself (per plan.md: port the math, not the library). Convention
// matches WPILib: x = forward, y = left, positive omega = counterclockwise.
#pragma once

#include <array>

#include "robot/robot_constants.hpp"

namespace kinematics {

struct ChassisSpeeds {
    double vx_mps = 0.0;           // forward
    double vy_mps = 0.0;           // strafe, positive left
    double omega_rad_per_s = 0.0;  // yaw rate, positive counterclockwise
};

struct ModuleState {
    double speed_mps = 0.0;
    double angle_rad = 0.0;
};

class SwerveKinematics {
public:
    explicit SwerveKinematics(std::array<robot::Translation2d, 4> module_positions);

    // Chassis velocity -> one ModuleState per module, same order as the
    // module_positions passed to the constructor. (plan.md's "forward
    // kinematics": chassis -> per-module.)
    [[nodiscard]] std::array<ModuleState, 4> to_module_states(const ChassisSpeeds &speeds) const;

    // Per-module measured (speed, angle) -> reconstructed chassis speed, via
    // least-squares solve of the same relationship (for odometry; plan.md's
    // "inverse kinematics": per-module -> chassis).
    [[nodiscard]] ChassisSpeeds to_chassis_speeds(const std::array<ModuleState, 4> &states) const;

    // Minimizes the steering delta for one module: if the desired angle is
    // more than 90 degrees from the current one, commands the flipped
    // angle (+180 deg) with reversed speed instead -- same trick WPILib's
    // SwerveModuleState::Optimize uses. Pure trig, no vendor assumptions,
    // so ported directly rather than re-derived.
    [[nodiscard]] static ModuleState optimize(const ModuleState &desired, double current_angle_rad);

private:
    std::array<robot::Translation2d, 4> module_positions_;
};

}  // namespace kinematics
