// Wire format for the ZeroMQ link between the C++ real-time core and the
// Python teleop/orchestration layer (plan.md Layer 4). Plain packed
// structs, not a serialization library -- deliberately minimal, matching
// this project's "thin wrapper, not a framework" approach elsewhere.
//
// Both ends are assumed to run on little-endian x86_64 (the LattePanda
// itself, or a laptop on the same LAN) -- this does not handle
// cross-architecture endianness. Revisit before trusting these numbers if
// that assumption ever breaks.
#pragma once

#include <cstdint>

namespace bridge {

constexpr int kCommandPort = 5555;    // Python -> C++: ChassisCommandWire
constexpr int kTelemetryPort = 5556;  // C++ -> Python: ChassisTelemetryWire

#pragma pack(push, 1)

struct ChassisCommandWire {
    double vx_mps;
    double vy_mps;
    double omega_rad_per_s;
};

struct ModuleTelemetryWire {
    double steer_angle_deg;   // measured, wheel frame (calibration offset already removed)
    double drive_speed_mps;   // measured
    std::uint8_t has_fault;   // this module's steer OR drive fault, 0 or 1
};

struct ChassisTelemetryWire {
    double vx_mps;            // reconstructed from real per-module feedback, not the command
    double vy_mps;
    double omega_rad_per_s;
    std::uint8_t any_fault;   // 0 or 1, whole-robot OR of every module's fault
    ModuleTelemetryWire modules[4];  // front-left, front-right, rear-left, rear-right
};

#pragma pack(pop)

static_assert(sizeof(ChassisCommandWire) == 24, "wire layout must stay packed/stable");
static_assert(sizeof(ModuleTelemetryWire) == 17, "wire layout must stay packed/stable");
static_assert(sizeof(ChassisTelemetryWire) == 93, "wire layout must stay packed/stable");

}  // namespace bridge
