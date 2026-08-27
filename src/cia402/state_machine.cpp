#include "cia402/state_machine.hpp"

#include "cia402/objects.hpp"

namespace cia402 {

namespace {

constexpr std::uint16_t kReady = kStatuswordMaskReadyToSwitchOn;
constexpr std::uint16_t kSwitchedOn = kStatuswordMaskSwitchedOn;
constexpr std::uint16_t kOpEnabled = kStatuswordMaskOperationEnabled;
constexpr std::uint16_t kFault = kStatuswordMaskFault;
constexpr std::uint16_t kQuickStop = kStatuswordMaskQuickStop;
constexpr std::uint16_t kSod = kStatuswordMaskSwitchOnDisabled;

constexpr std::uint16_t kMaskNoQuickStopBit = kReady | kSwitchedOn | kOpEnabled | kFault | kSod;
constexpr std::uint16_t kMaskWithQuickStopBit = kMaskNoQuickStopBit | kQuickStop;

struct DecodeEntry {
    std::uint16_t mask;
    std::uint16_t value;
    DriveState state;
};

// (mask, value, state) checked in order -- per the standard DS402
// statusword decode table (bits 0,1,2,3,5,6).
constexpr DecodeEntry kDecodeTable[] = {
    {kMaskNoQuickStopBit, 0x00, DriveState::NotReadyToSwitchOn},
    {kMaskNoQuickStopBit, kSod, DriveState::SwitchOnDisabled},
    {kMaskWithQuickStopBit, kReady | kQuickStop, DriveState::ReadyToSwitchOn},
    {kMaskWithQuickStopBit, kReady | kSwitchedOn | kQuickStop, DriveState::SwitchedOn},
    {kMaskWithQuickStopBit, kReady | kSwitchedOn | kOpEnabled | kQuickStop, DriveState::OperationEnabled},
    {kMaskWithQuickStopBit, kReady | kSwitchedOn | kOpEnabled, DriveState::QuickStopActive},
    {kMaskNoQuickStopBit, kReady | kSwitchedOn | kOpEnabled | kFault, DriveState::FaultReactionActive},
    {kMaskNoQuickStopBit, kFault, DriveState::Fault},
};

}  // namespace

std::string_view to_string(DriveState state) {
    switch (state) {
        case DriveState::NotReadyToSwitchOn: return "NOT_READY_TO_SWITCH_ON";
        case DriveState::SwitchOnDisabled: return "SWITCH_ON_DISABLED";
        case DriveState::ReadyToSwitchOn: return "READY_TO_SWITCH_ON";
        case DriveState::SwitchedOn: return "SWITCHED_ON";
        case DriveState::OperationEnabled: return "OPERATION_ENABLED";
        case DriveState::QuickStopActive: return "QUICK_STOP_ACTIVE";
        case DriveState::FaultReactionActive: return "FAULT_REACTION_ACTIVE";
        case DriveState::Fault: return "FAULT";
    }
    return "UNKNOWN";
}

DriveState decode_statusword(std::uint16_t statusword) {
    for (const auto &entry : kDecodeTable) {
        if ((statusword & entry.mask) == entry.value) {
            return entry.state;
        }
    }
    // Should not happen for a spec-compliant drive; treat as not-ready
    // rather than raising, so a single malformed PDO cycle doesn't crash
    // the caller's control loop.
    return DriveState::NotReadyToSwitchOn;
}

void StateMachine::update(std::uint16_t statusword) {
    state_ = decode_statusword(statusword);
    if (state_ != DriveState::Fault) {
        fault_reset_pulse_pending_ = false;
        fault_reset_pulse_sent_ = false;
    }
}

void StateMachine::request_enable() { target_ = Target::Enable; }
void StateMachine::request_disable() { target_ = Target::Disable; }
void StateMachine::request_quick_stop() { target_ = Target::QuickStop; }

void StateMachine::request_fault_reset() {
    fault_reset_pulse_pending_ = true;
    fault_reset_pulse_sent_ = false;
}

std::uint16_t StateMachine::next_controlword_bits() {
    if (state_ == DriveState::Fault || state_ == DriveState::FaultReactionActive) {
        if (fault_reset_pulse_pending_ && !fault_reset_pulse_sent_) {
            fault_reset_pulse_sent_ = true;
            return static_cast<std::uint16_t>(ControlwordCommand::FaultReset);
        }
        // Either no reset requested yet, or the rising-edge pulse was
        // already sent last cycle -- drop the bit back to 0 either way.
        return 0x00;
    }

    switch (target_) {
        case Target::QuickStop:
            switch (state_) {
                case DriveState::OperationEnabled:
                case DriveState::SwitchedOn:
                case DriveState::ReadyToSwitchOn:
                case DriveState::QuickStopActive:
                    return static_cast<std::uint16_t>(ControlwordCommand::QuickStop);
                default:
                    return static_cast<std::uint16_t>(ControlwordCommand::DisableVoltage);
            }
        case Target::Disable:
            return static_cast<std::uint16_t>(ControlwordCommand::DisableVoltage);
        case Target::Enable:
            switch (state_) {
                case DriveState::SwitchOnDisabled:
                    return static_cast<std::uint16_t>(ControlwordCommand::Shutdown);
                case DriveState::ReadyToSwitchOn:
                    return static_cast<std::uint16_t>(ControlwordCommand::SwitchOn);
                case DriveState::SwitchedOn:
                case DriveState::OperationEnabled:
                    return static_cast<std::uint16_t>(ControlwordCommand::EnableOperation);
                case DriveState::QuickStopActive:
                    // Drop to SWITCH_ON_DISABLED first, then climb normally.
                    return static_cast<std::uint16_t>(ControlwordCommand::DisableVoltage);
                default:
                    // NOT_READY_TO_SWITCH_ON: drive self-transitions per
                    // DS402; wait.
                    return 0x00;
            }
        case Target::None:
        default:
            return 0x00;
    }
}

}  // namespace cia402
