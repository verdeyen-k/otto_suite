// Physical constants and runtime-tunable limits for this specific chassis.
// Loaded at runtime from a flat "key: value" file (see
// config/robot_constants.yaml) via load_robot_config() -- a dimension or
// limit change is an edit to that file, not a rebuild.
//
// Everything is in SI units (meters, radians except where "_deg" is in the
// name) -- the kinematics layer (Layer 3, plan.md) works in
// ChassisSpeeds/ModuleState terms (m/s, rad), same as WPILib's
// SwerveDriveKinematics; unit conversion to encoder counts/degrees happens
// at the driver boundary, not here.
#pragma once

#include <array>
#include <cmath>
#include <string>

namespace robot {

// x = forward, y = left -- same convention the kinematics module and
// WPILib's SwerveDriveKinematics use, so module positions plug in directly.
struct Translation2d {
    double x_m;
    double y_m;
};

// Fixed module order used everywhere a per-module array is indexed
// (kinematics output, driver assignment, steer offsets below): front-left,
// front-right, rear-left, rear-right.
enum ModuleIndex { kFrontLeft = 0, kFrontRight = 1, kRearLeft = 2, kRearRight = 3 };

// Default location tools look for the config file, relative to the
// current working directory (the run commands in SESSION_NOTES.md all run
// from the repo root).
constexpr const char *kDefaultConfigPath = "config/robot_constants.yaml";

// Which EtherCAT slave (and, for drive, which Copley axis A/B) plays each
// module's steer/drive role. drive_axis is 'a' or 'b' -- kept as a plain
// char rather than copley::Axis so this header stays free of the
// EtherCAT/CiA-402 driver layer, same separation kinematics.hpp keeps.
struct ModuleBusLocation {
    int steer_slave;
    int drive_slave;
    char drive_axis;
};

struct RobotConfig {
    double wheelbase_width_m;
    double wheelbase_length_m;
    double wheel_diameter_m;

    // Drive wheel encoder (Copley BE2 motor-side feedback on this chassis).
    int drive_encoder_counts_per_rev;
    double drive_gear_ratio;  // motor revolutions per wheel revolution

    // Whether this module's drive actuator's positive raw-velocity
    // direction is physically reversed relative to the wheel-frame
    // convention (positive = forward) -- a mounting fact, not a
    // calibration number, same reasoning as steer_invert below. Applied by
    // whichever code converts between wheel_mps and raw drive counts/s.
    // Indexed by ModuleIndex. Uncalibrated chassis: leave false.
    std::array<bool, 4> drive_invert{false, false, false, false};

    double max_speed_mps;
    double max_omega_deg_s;
    double max_steer_rate_deg_s;
    double max_steer_accel_deg_s2;
    double max_accel_mps2;

    // Hard ceiling on any single drive wheel's own rotational speed (wheel
    // RPM, not motor RPM -- independent of drive_gear_ratio) -- distinct
    // from max_speed_mps/max_omega_deg_s above, which are chassis-level
    // targets: a combined translate+rotate command can require one wheel
    // to spin faster than either implies on its own. When that happens,
    // every module's speed is uniformly derated (see
    // SwerveKinematics::desaturate_wheel_speeds) so the fastest wheel
    // never exceeds this, while keeping the commanded heading correct
    // (uniform scaling changes no module's angle).
    double max_wheel_speed_rpm;

    double max_wheel_speed_mps() const {
        return max_wheel_speed_rpm / 60.0 * 2.0 * M_PI * wheel_radius_m();
    }

    // Raw actuator degrees when this module's wheel is physically pointing
    // forward (wheel-frame angle 0) -- read directly off zeroerr_state (or
    // the actuator's own telemetry) with the wheel manually set to
    // forward, no sign change needed. Indexed by ModuleIndex. Uncalibrated
    // chassis: leave at 0.
    std::array<double, 4> steer_angle_offset_deg{0.0, 0.0, 0.0, 0.0};

    // Whether this module's steer actuator's positive raw-degree direction
    // is physically reversed relative to the wheel-frame convention
    // (kinematics.hpp: CCW positive, matching WPILib) -- a mounting fact,
    // not a calibration number: symmetric swerve builds commonly need
    // opposite sign conventions on left vs. right (or front vs. rear)
    // modules for "increasing raw degrees" to mean the same physical
    // rotation sense the kinematics math assumes. Translation commands
    // (all modules at the same angle, often 0) can look correct even with
    // this wrong; rotation is what exposes it, since it needs each module
    // at a different, correctly-signed angle -- wheels forming an "X"
    // instead of the expected tangential pinwheel during rotate-in-place
    // is the signature. Indexed by ModuleIndex. Uncalibrated chassis:
    // leave false.
    std::array<bool, 4> steer_invert{false, false, false, false};

    // wheel-frame degrees (kinematics convention) <-> raw actuator
    // degrees, composing steer_angle_offset_deg and steer_invert.
    // Offset is unaffected by invert: it was calibrated by reading the
    // raw value with the wheel already at wheel-frame 0, where the
    // invert's sign multiplier is a no-op either way.
    double wheel_angle_to_raw_deg(std::size_t module, double wheel_angle_deg) const {
        return steer_angle_offset_deg[module] + (steer_invert[module] ? -wheel_angle_deg : wheel_angle_deg);
    }
    double raw_to_wheel_angle_deg(std::size_t module, double raw_deg) const {
        const double delta = raw_deg - steer_angle_offset_deg[module];
        return steer_invert[module] ? -delta : delta;
    }

    // Same steer_invert sign flip as wheel_angle_to_raw_deg, without the
    // offset -- a rate has no absolute reference point to offset. For
    // converting a wheel-frame angular velocity (e.g. a host-side steer
    // position loop's output) into the raw actuator's sign convention.
    double wheel_angular_velocity_to_raw_deg_s(std::size_t module, double wheel_deg_s) const {
        return steer_invert[module] ? -wheel_deg_s : wheel_deg_s;
    }

    // Indexed by ModuleIndex. Not independently verified against
    // docs/ecatbustopo.md's physical layout notes -- those two have
    // disagreed before (topology doc said slave 6, live testing showed
    // slave 7). If slave numbers ever seem off, re-scan (slaveinfo or any
    // tool's own scan output) rather than trusting either document.
    std::array<ModuleBusLocation, 4> module_bus_locations;

    double wheel_radius_m() const { return wheel_diameter_m / 2.0; }

    std::array<Translation2d, 4> module_positions() const {
        return {
            Translation2d{wheelbase_length_m / 2.0, wheelbase_width_m / 2.0},    // front-left
            Translation2d{wheelbase_length_m / 2.0, -wheelbase_width_m / 2.0},   // front-right
            Translation2d{-wheelbase_length_m / 2.0, wheelbase_width_m / 2.0},   // rear-left
            Translation2d{-wheelbase_length_m / 2.0, -wheelbase_width_m / 2.0},  // rear-right
        };
    }
};

// Parses `path` as a flat "key: value" file -- one scalar per line, blank
// lines and '#' comments ignored, no nesting. Throws std::runtime_error
// (naming the path and, where possible, the bad line or missing key) on
// any parse failure or missing required key.
RobotConfig load_robot_config(const std::string &path);

}  // namespace robot
