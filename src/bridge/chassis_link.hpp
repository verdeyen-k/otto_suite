// Thin C++ wrapper around libzmq's C API (zmq.h) for the command/telemetry
// link to the Python teleop layer -- same "thin wrapper, not a framework"
// approach as ethercat/soem_master.hpp. Deliberately not using cppzmq
// (header-only C++ bindings): libzmq's C API is small and stable enough
// that a raw wrapper is simpler than adding another vendored dependency.
//
// The C++ real-time core BINDS both sockets (it's the long-lived process
// tied to the hardware); Python CONNECTs to both. The command socket is a
// SUB with ZMQ_CONFLATE set, so at most one message is ever queued -- a
// backlogged/stale command can never accumulate and get processed late.
//
// This class only moves ChassisSpeeds in and out; it does NOT implement
// the "hold last command, then ramp to zero, then disable" watchdog
// discussed for a dropped/stale link -- that's real-time-loop policy
// (needs the loop's own cycle timing) and belongs in whatever hardware
// driver tool consumes try_receive_command(), not here.
#pragma once

#include <array>
#include <optional>

#include "kinematics/swerve_kinematics.hpp"
#include "bridge/messages.hpp"

namespace bridge {

// In-memory (unpacked) counterpart to ModuleTelemetryWire -- see
// publish_telemetry.
struct ModuleTelemetry {
    double angle_deg = 0.0;
    double speed_mps = 0.0;
    bool has_fault = false;
};

class ChassisLink {
public:
    // Binds tcp://*:command_port (SUB) and tcp://*:telemetry_port (PUB).
    // Throws std::runtime_error if either socket fails to bind.
    explicit ChassisLink(int command_port = kCommandPort, int telemetry_port = kTelemetryPort);
    ~ChassisLink();

    ChassisLink(const ChassisLink &) = delete;
    ChassisLink &operator=(const ChassisLink &) = delete;

    // Non-blocking. Returns the latest command if a new one has arrived
    // since the last call, std::nullopt otherwise -- either nothing
    // waiting, or a malformed message (logged and ignored rather than
    // crashing the real-time loop over one bad packet).
    [[nodiscard]] std::optional<kinematics::ChassisSpeeds> try_receive_command();

    // Non-blocking, fire-and-forget: never worth blocking the real-time
    // loop over a slow or absent subscriber. modules is FL/FR/RL/RR order,
    // same as everywhere else a per-module array is indexed.
    void publish_telemetry(const kinematics::ChassisSpeeds &speeds, bool any_fault,
                            const std::array<ModuleTelemetry, 4> &modules);

private:
    void *context_ = nullptr;
    void *command_sub_ = nullptr;
    void *telemetry_pub_ = nullptr;
};

}  // namespace bridge
