#include "zeroerr/zeroerr_actuator.hpp"

#include <cmath>
#include <cstring>

#include "cia402/objects.hpp"
#include "zeroerr/zeroerr_identity.hpp"

namespace zeroerr {

namespace {

constexpr double kDegPerRev = 360.0;

// Generous upper bound (in update() calls, i.e. PDO cycles -- this class
// doesn't know the caller's actual cycle period) on waiting for the
// drive's Set-point Acknowledge to rise or clear. The manual's own timing
// diagrams (Fig. 5-4/5-5) show it arriving essentially immediately after
// the rising edge, so this should never actually bind in practice; it
// exists only so a drive that never acknowledges (or never clears)
// doesn't freeze this actuator's steering forever -- at a typical 5ms
// cycle this is roughly 1 second.
constexpr int kAckTimeoutCycles = 200;

template <typename T>
void write_field(ethercat::SoemMaster &master, int slave_index, int offset, T value) {
    master.write_output_bytes(slave_index, offset, reinterpret_cast<const std::uint8_t *>(&value), sizeof(T));
}

template <typename T>
T read_field(const ethercat::SoemMaster &master, int slave_index, int offset) {
    T value{};
    master.read_input_bytes(slave_index, offset, reinterpret_cast<std::uint8_t *>(&value), sizeof(T));
    return value;
}

}  // namespace

double counts_to_deg(std::int32_t counts) {
    return static_cast<double>(counts) / kEncoderCountsPerRev * kDegPerRev;
}

std::int32_t deg_to_counts(double deg) {
    return static_cast<std::int32_t>(std::lround(deg / kDegPerRev * kEncoderCountsPerRev));
}

void configure_zeroerr_pdos(ethercat::SoemMaster &master, int slave_index, double profile_velocity_deg_s,
                             double profile_accel_deg_s2, double profile_decel_deg_s2) {
    master.set_config_func(slave_index, [profile_velocity_deg_s, profile_accel_deg_s2,
                                          profile_decel_deg_s2](ethercat::SoemMaster &m, int idx) {
        const std::vector<ethercat::PdoMapEntry> rx_entries = {
            {0x607A, 0, 32},  // Target position
            {0x60FE, 0, 32},  // Digital outputs
            {0x6040, 0, 16},  // Controlword
            {0x60FF, 0, 32},  // Target velocity
            {0x6071, 0, 16},  // Target torque
        };
        ethercat::map_pdo(m, idx, kRxPdoIndex, rx_entries);

        const std::vector<ethercat::PdoMapEntry> tx_entries = {
            {0x6064, 0, 32},  // Position actual value
            {0x606C, 0, 32},  // Velocity actual value
            {0x6077, 0, 16},  // Torque actual value
            {0x60FD, 0, 32},  // Digital inputs
            {0x6041, 0, 16},  // Statusword
            {0x6061, 0, 16},  // Modes of operation display
        };
        ethercat::map_pdo(m, idx, kTxPdoIndex, tx_entries);

        ethercat::assign_single_pdo(m, idx, 0x1C12, kRxPdoIndex);
        ethercat::assign_single_pdo(m, idx, 0x1C13, kTxPdoIndex);

        auto mode = static_cast<std::int8_t>(cia402::ModeOfOperation::ProfilePosition);
        m.sdo_write(idx, cia402::kModesOfOperation, 0, &mode, sizeof(mode));

        // Profile Velocity/Acceleration/Deceleration (plus/s, plus/s^2) --
        // deg_to_counts()'s scale factor (counts per revolution / 360) is
        // the same linear conversion regardless of whether the input is a
        // position, a velocity, or an acceleration, so it's reused as-is.
        auto profile_velocity = static_cast<std::uint32_t>(deg_to_counts(profile_velocity_deg_s));
        auto profile_accel = static_cast<std::uint32_t>(deg_to_counts(profile_accel_deg_s2));
        auto profile_decel = static_cast<std::uint32_t>(deg_to_counts(profile_decel_deg_s2));
        m.sdo_write(idx, kProfileVelocityIndex, 0, &profile_velocity, sizeof(profile_velocity));
        m.sdo_write(idx, kProfileAccelerationIndex, 0, &profile_accel, sizeof(profile_accel));
        m.sdo_write(idx, kProfileDecelerationIndex, 0, &profile_decel, sizeof(profile_decel));
    });
}

ZeroErrActuator::ZeroErrActuator(ethercat::SoemMaster &master, int slave_index)
    : master_(master), slave_index_(slave_index) {}

std::optional<std::uint16_t> ZeroErrActuator::read_error_code_live() const {
    std::uint16_t value = 0;
    int wkc = master_.sdo_read(slave_index_, cia402::kErrorCode, 0, &value, sizeof(value));
    if (wkc <= 0) {
        return std::nullopt;
    }
    return value;
}

namespace {
constexpr std::uint16_t kSm2ParametersIndex = 0x1C32;  // outputs
constexpr std::uint16_t kSm3ParametersIndex = 0x1C33;  // inputs
constexpr std::uint8_t kSmEventMissedSubindex = 0x0C;
constexpr std::uint8_t kSyncErrorSubindex = 0x20;
}  // namespace

std::optional<std::uint16_t> ZeroErrActuator::read_sm_event_missed(bool outputs) const {
    std::uint16_t value = 0;
    int wkc = master_.sdo_read(slave_index_, outputs ? kSm2ParametersIndex : kSm3ParametersIndex,
                                kSmEventMissedSubindex, &value, sizeof(value));
    if (wkc <= 0) {
        return std::nullopt;
    }
    return value;
}

std::optional<bool> ZeroErrActuator::read_sync_error(bool outputs) const {
    std::uint8_t value = 0;
    int wkc = master_.sdo_read(slave_index_, outputs ? kSm2ParametersIndex : kSm3ParametersIndex,
                                kSyncErrorSubindex, &value, sizeof(value));
    if (wkc <= 0) {
        return std::nullopt;
    }
    return value != 0;
}

void ZeroErrActuator::set_target_angle_deg(double target_deg) { pending_target_counts_ = deg_to_counts(target_deg); }

void ZeroErrActuator::update() {
    last_statusword_ = read_field<std::uint16_t>(master_, slave_index_, pdo_layout::kStatuswordOffset);
    const bool was_faulted = fsm_.has_fault();
    fsm_.update(last_statusword_);

    if (fsm_.has_fault() && !was_faulted) {
        // SDO/mailbox read -- only on the fault edge, not every cycle.
        int wkc = master_.sdo_read(slave_index_, cia402::kErrorCode, 0, &last_error_code_, sizeof(last_error_code_));
        last_error_code_read_failed_ = wkc <= 0;
        if (last_error_code_read_failed_) {
            last_error_code_ = 0;  // don't leave a stale value from a previous fault
        }
    } else if (!fsm_.has_fault()) {
        last_error_code_ = 0;
        last_error_code_read_failed_ = false;
    }

    last_position_counts_ = read_field<std::int32_t>(master_, slave_index_, pdo_layout::kPositionActualOffset);
    last_velocity_counts_per_s_ = read_field<std::int32_t>(master_, slave_index_, pdo_layout::kVelocityActualOffset);
    last_effort_raw_ = read_field<std::int16_t>(master_, slave_index_, pdo_layout::kEffortActualOffset);
    last_digital_inputs_ = read_field<std::uint32_t>(master_, slave_index_, pdo_layout::kDigitalInputsOffset);
    last_mode_of_operation_display_ =
        read_field<std::uint16_t>(master_, slave_index_, pdo_layout::kModeOfOperationDisplayOffset);

    std::uint16_t controlword = fsm_.next_controlword_bits();
    // Safe default: hold the target at the actual measured position --
    // otherwise a drive reaching OPERATION_ENABLED with a stale/zero
    // target PDO field would see a huge implied move on its first cycle.
    std::int32_t target_counts_to_write = last_position_counts_;

    if (!fsm_.is_operational()) {
        // Not enabled: reset the move-triggering state so the next enable
        // starts with a clean 0 -> 1 edge instead of resuming mid-toggle
        // (bit4/bit5 are meaningless outside OPERATION_ENABLED anyway --
        // fsm_'s own bits fully own the controlword here).
        bit4_high_ = false;
        awaiting_ack_ = false;
        awaiting_ack_clear_ = false;
        ack_wait_cycles_ = 0;
        triggered_target_counts_.reset();
    } else if (pending_target_counts_.has_value()) {
        target_counts_to_write = *pending_target_counts_;
        // Change Set Immediately: a retarget while already moving blends
        // into the new target using Profile Acceleration/Deceleration
        // instead of decelerating to a stop at the old one first (manual
        // Table 5-9/Fig. 5-4 vs 5-5, p.58-60) -- what continuous steering
        // tracking needs. See set_change_set_immediately()'s comment for
        // why this is conditional, not unconditional as the manual's
        // demonstrated use case alone would suggest.
        if (change_set_immediately_) {
            controlword |= kControlwordBitChangeSetImmediately;
        }
        const bool ack = (last_statusword_ & kStatuswordBitSetpointAck) != 0;

        if (awaiting_ack_clear_) {
            // bit4 already dropped; wait for the drive's own acknowledge
            // bit to clear before it's safe to raise bit4 again for a new
            // target -- see the field comment in the header for why this
            // matters (skipping it is what silently dropped updates).
            if (!ack || ++ack_wait_cycles_ > kAckTimeoutCycles) {
                awaiting_ack_clear_ = false;
            }
        } else if (awaiting_ack_) {
            // bit4 held high; wait for the drive to acknowledge this
            // target before touching bit4 again.
            if (ack) {
                bit4_high_ = false;
                awaiting_ack_ = false;
                awaiting_ack_clear_ = true;
                ack_wait_cycles_ = 0;
                triggered_target_counts_ = pending_target_counts_;
            } else if (++ack_wait_cycles_ > kAckTimeoutCycles) {
                bit4_high_ = false;
                awaiting_ack_ = false;
                ack_wait_cycles_ = 0;
            }
        } else if (triggered_target_counts_ != pending_target_counts_) {
            bit4_high_ = true;
            awaiting_ack_ = true;
            ack_wait_cycles_ = 0;
        }

        if (bit4_high_) {
            controlword |= kControlwordBitNewSetpoint;
        }
    }

    last_controlword_ = controlword;
    write_field<std::uint16_t>(master_, slave_index_, pdo_layout::kControlwordOffset, last_controlword_);
    write_field<std::int32_t>(master_, slave_index_, pdo_layout::kTargetPositionOffset, target_counts_to_write);

    last_digital_outputs_ = fsm_.wants_enable() ? kDigitalOutputsBrakeReleaseBit : 0;
    write_field<std::uint32_t>(master_, slave_index_, pdo_layout::kDigitalOutputsOffset, last_digital_outputs_);
}

StateSnapshot ZeroErrActuator::snapshot() const {
    StateSnapshot s{};
    s.state = fsm_.state();
    s.statusword_raw = last_statusword_;
    s.controlword_raw = last_controlword_;
    s.digital_outputs_raw = last_digital_outputs_;
    s.digital_inputs_raw = last_digital_inputs_;
    s.position_counts = last_position_counts_;
    s.position_deg = counts_to_deg(last_position_counts_);
    s.velocity_actual_counts_per_s = last_velocity_counts_per_s_;
    s.velocity_deg_per_s = counts_to_deg(last_velocity_counts_per_s_);
    s.effort_actual_raw = last_effort_raw_;
    s.mode_of_operation_display = last_mode_of_operation_display_;
    s.error_code = last_error_code_;
    s.has_fault = fsm_.has_fault();
    s.sto_active = last_error_code_ == cia402::kZeroErrStoErrorCode;
    s.error_code_read_failed = last_error_code_read_failed_;
    return s;
}

}  // namespace zeroerr
