// Physical constants for this specific chassis. Kept separate from any
// driver/kinematics code so a dimension change (or a future chassis
// revision) is a one-line edit here rather than a hunt through math code.
//
// Everything is in SI units (meters) -- the kinematics layer (Layer 3,
// plan.md) works in ChassisSpeeds/ModuleState terms (m/s, rad), same as
// WPILib's SwerveDriveKinematics; unit conversion to encoder counts/degrees
// happens at the driver boundary, not here.
#pragma once

namespace robot {

// Distance between the left and right wheel contact points (measured
// track width), in meters.
constexpr double kWheelbaseWidthM = 0.475;

// Distance between the front and rear wheel contact points (measured
// wheelbase length), in meters.
constexpr double kWheelbaseLengthM = 0.625;

constexpr double kWheelDiameterM = 0.170;
constexpr double kWheelRadiusM = kWheelDiameterM / 2.0;

// x = forward, y = left -- same convention the kinematics module and
// WPILib's SwerveDriveKinematics use, so module positions plug in directly.
struct Translation2d {
    double x_m;
    double y_m;
};

// Fixed module geometry for this chassis, in a consistent order (front-left,
// front-right, rear-left, rear-right). Anything indexing into this array
// (kinematics output, per-module driver assignment) must agree on this
// order.
constexpr Translation2d kModulePositions[4] = {
    {kWheelbaseLengthM / 2.0, kWheelbaseWidthM / 2.0},    // front-left
    {kWheelbaseLengthM / 2.0, -kWheelbaseWidthM / 2.0},   // front-right
    {-kWheelbaseLengthM / 2.0, kWheelbaseWidthM / 2.0},   // rear-left
    {-kWheelbaseLengthM / 2.0, -kWheelbaseWidthM / 2.0},  // rear-right
};

// Drive wheel encoder (Copley BE2 motor-side feedback on this chassis):
// 2^19 counts/rev, direct drive (no gearbox/belt reduction between motor
// and wheel) -- per-robot facts confirmed directly, not derived from the
// BE2 manual (which doesn't fix an encoder type or drivetrain ratio).
constexpr int kDriveEncoderCountsPerRev = 524288;
constexpr double kDriveGearRatio = 1.0;  // motor revolutions per wheel revolution

}  // namespace robot
