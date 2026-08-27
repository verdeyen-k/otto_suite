// Generic CiA 402 (DS402) drive state machine. Mode-agnostic (works
// identically for a CSP-mode steering actuator and a CSV-mode drive
// actuator) since the DS402 controlword/statusword state machine is part
// of the standard itself, independent of vendor or control mode.
#pragma once

#include <cstdint>
#include <optional>
#include <string_view>

namespace cia402 {

enum class DriveState {
    NotReadyToSwitchOn,
    SwitchOnDisabled,
    ReadyToSwitchOn,
    SwitchedOn,
    OperationEnabled,
    QuickStopActive,
    FaultReactionActive,
    Fault,
};

std::string_view to_string(DriveState state);

DriveState decode_statusword(std::uint16_t statusword);

// Drives one axis through the DS402 controlword/statusword FSM.
//
// Usage each PDO cycle:
//   DriveState state = fsm.update(statusword_from_bus);
//   std::uint16_t controlword_to_write = fsm.next_controlword_bits();
class StateMachine {
public:
    void update(std::uint16_t statusword);

    void request_enable();
    void request_disable();
    void request_quick_stop();
    void request_fault_reset();

    [[nodiscard]] DriveState state() const { return state_; }
    [[nodiscard]] bool is_operational() const { return state_ == DriveState::OperationEnabled; }
    [[nodiscard]] bool has_fault() const {
        return state_ == DriveState::Fault || state_ == DriveState::FaultReactionActive;
    }

    // True from the moment enable is requested (climbing the DS402 chain)
    // through OPERATION_ENABLED -- used by ZeroErrActuator to release the
    // eRob's holding brake proactively, rather than only after
    // OPERATION_ENABLED is already confirmed.
    [[nodiscard]] bool wants_enable() const { return target_ == Target::Enable; }

    // Mutates internal fault-reset-pulse tracking as a side effect (the
    // rising-edge pulse must be sent exactly once) -- call exactly once per
    // PDO cycle, after update().
    [[nodiscard]] std::uint16_t next_controlword_bits();

private:
    enum class Target { None, Enable, Disable, QuickStop };

    DriveState state_ = DriveState::NotReadyToSwitchOn;
    Target target_ = Target::None;
    bool fault_reset_pulse_pending_ = false;
    bool fault_reset_pulse_sent_ = false;
};

}  // namespace cia402
