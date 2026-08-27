#include "zeroerr/zeroerr_actuator.hpp"

#include <cmath>
#include <cstring>

#include "cia402/objects.hpp"
#include "zeroerr/zeroerr_identity.hpp"

namespace zeroerr {

namespace {

constexpr double kDegPerRev = 360.0;

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

void configure_zeroerr_pdos(ethercat::SoemMaster &master, int slave_index) {
    master.set_config_func(slave_index, [](ethercat::SoemMaster &m, int idx) {
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

        auto mode = static_cast<std::int8_t>(cia402::ModeOfOperation::CyclicSyncPosition);
        m.sdo_write(idx, cia402::kModesOfOperation, 0, &mode, sizeof(mode));
    });
}

ZeroErrActuator::ZeroErrActuator(ethercat::SoemMaster &master, int slave_index)
    : master_(master), slave_index_(slave_index) {}

void ZeroErrActuator::set_target_angle_deg(double target_deg) {
    last_commanded_deg_ = target_deg;
    write_field<std::int32_t>(master_, slave_index_, pdo_layout::kTargetPositionOffset, deg_to_counts(target_deg));
}

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

    last_controlword_ = fsm_.next_controlword_bits();
    write_field<std::uint16_t>(master_, slave_index_, pdo_layout::kControlwordOffset, last_controlword_);

    last_digital_outputs_ = fsm_.wants_enable() ? kDigitalOutputsBrakeReleaseBit : 0;
    write_field<std::uint32_t>(master_, slave_index_, pdo_layout::kDigitalOutputsOffset, last_digital_outputs_);

    last_position_counts_ = read_field<std::int32_t>(master_, slave_index_, pdo_layout::kPositionActualOffset);
    last_velocity_counts_per_s_ = read_field<std::int32_t>(master_, slave_index_, pdo_layout::kVelocityActualOffset);
    last_effort_raw_ = read_field<std::int16_t>(master_, slave_index_, pdo_layout::kEffortActualOffset);
    last_digital_inputs_ = read_field<std::uint32_t>(master_, slave_index_, pdo_layout::kDigitalInputsOffset);
    last_mode_of_operation_display_ =
        read_field<std::uint16_t>(master_, slave_index_, pdo_layout::kModeOfOperationDisplayOffset);

    if (!last_commanded_deg_.has_value()) {
        // Before the first real set_target_angle_deg() call, hold the
        // target at the actual measured position -- otherwise a CSP drive
        // reaching OPERATION_ENABLED with a stale/zero target PDO field
        // sees a huge instantaneous following error and faults
        // immediately.
        write_field<std::int32_t>(master_, slave_index_, pdo_layout::kTargetPositionOffset, last_position_counts_);
    }
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
