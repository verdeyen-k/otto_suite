#include "copley/copley_axis.hpp"

#include <chrono>
#include <cstdio>
#include <thread>

#include "cia402/objects.hpp"

namespace copley {

namespace {

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

// The manual explicitly warns (p.64): "there may be some delay between
// setting the mode of operation and the drive assuming that mode. To read
// the active mode of operation, use object 0x6061." Desired State (0x2300)
// switches the amplifier's control source at the same time -- polling the
// Mode of Operation Display confirms both have actually settled before
// this axis is handed off to the CiA-402 enable sequence. Proceeding
// straight through OPERATIONAL into an enable attempt while the drive is
// still mid-switchover is a plausible cause of a spurious 0x61FF "Command
// error" on the very first enable, since the SDO writes that requested the
// switch return success (accepted) well before the switch is complete.
bool wait_for_mode_display(ethercat::SoemMaster &master, int slave_index, std::uint16_t axis_offset,
                            std::int8_t expected_mode) {
    for (int i = 0; i < 50; ++i) {
        std::int8_t display = 0;
        int wkc = master.sdo_read(slave_index, cia402::kModesOfOperationDisplay + axis_offset, 0, &display,
                                   sizeof(display));
        if (wkc > 0 && display == expected_mode) {
            return true;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    return false;
}

}  // namespace

void configure_copley_pdos(ethercat::SoemMaster &master, int slave_index) {
    master.set_config_func(slave_index, [](ethercat::SoemMaster &m, int idx) {
        constexpr std::uint16_t kAxisBOffset = 0x0800;

        const std::vector<ethercat::PdoMapEntry> axis_b_rx = {
            {static_cast<std::uint16_t>(cia402::kControlword + kAxisBOffset), 0, 16},  // 0x6840 Controlword
            {static_cast<std::uint16_t>(0x60FF + kAxisBOffset), 0, 32},                // 0x68FF Target velocity
            {static_cast<std::uint16_t>(0x60B2 + kAxisBOffset), 0, 16},                // 0x68B2 Torque offset
        };
        ethercat::map_pdo(m, idx, kAxisBRxPdoIndex, axis_b_rx);

        const std::vector<ethercat::PdoMapEntry> axis_b_tx = {
            {static_cast<std::uint16_t>(cia402::kStatusword + kAxisBOffset), 0, 16},  // 0x6841 Statusword
            {static_cast<std::uint16_t>(0x6064 + kAxisBOffset), 0, 32},               // 0x6864 Actual position
            {static_cast<std::uint16_t>(0x60F4 + kAxisBOffset), 0, 32},               // 0x68F4 Following error
            {static_cast<std::uint16_t>(0x606C + kAxisBOffset), 0, 32},               // 0x686C Actual velocity
            {static_cast<std::uint16_t>(0x6077 + kAxisBOffset), 0, 16},               // 0x6877 Torque actual
        };
        ethercat::map_pdo(m, idx, kAxisBTxPdoIndex, axis_b_tx);

        // Switches Axis A off the CSP-mode default (fixed RxPDO 0x1700)
        // onto the CSV fixed RxPDO 0x1701, and combines it with Axis B's
        // just-mapped custom PDO in the same sync manager -- see
        // copley_identity.hpp for why this combined assignment is a
        // documented capability (0x1C12/0x1C13 support 0-4 entries), not
        // an undocumented trick.
        ethercat::assign_pdos(m, idx, kSm2RxAssignmentIndex, {kAxisAFixedRxPdoIndex, kAxisBRxPdoIndex});
        ethercat::assign_pdos(m, idx, kSm3TxAssignmentIndex, {kAxisAFixedTxPdoIndex, kAxisBTxPdoIndex});

        auto checked = [idx](ethercat::SoemMaster &mm, std::uint16_t index, const void *data, int size,
                              const char *what) {
            int wkc = mm.sdo_write(idx, index, 0, data, size);
            if (wkc <= 0) {
                std::fprintf(stderr, "configure_copley_pdos: SDO write FAILED (wkc=%d) slave=%d index=0x%04X (%s)\n",
                             wkc, idx, index, what);
            }
        };

        std::uint16_t desired_state = kDesiredStateCanOpenControlsAmplifier;
        checked(m, kDesiredStateIndex, &desired_state, sizeof(desired_state), "Desired State, axis A");
        checked(m, kDesiredStateIndex + kAxisBOffset, &desired_state, sizeof(desired_state), "Desired State, axis B");

        auto mode = static_cast<std::int8_t>(cia402::ModeOfOperation::CyclicSyncVelocity);
        checked(m, cia402::kModesOfOperation, &mode, sizeof(mode), "Modes of Operation, axis A");
        checked(m, cia402::kModesOfOperation + kAxisBOffset, &mode, sizeof(mode), "Modes of Operation, axis B");

        if (!wait_for_mode_display(m, idx, 0x0000, mode)) {
            std::fprintf(stderr,
                         "configure_copley_pdos: axis A Mode of Operation Display (0x6061) did not settle to "
                         "CSV (9) within 500ms -- proceeding anyway, but an enable attempt may be rejected\n");
        }
        if (!wait_for_mode_display(m, idx, kAxisBOffset, mode)) {
            std::fprintf(stderr,
                         "configure_copley_pdos: axis B Mode of Operation Display (0x6861) did not settle to "
                         "CSV (9) within 500ms -- proceeding anyway, but an enable attempt may be rejected\n");
        }
    });
}

CopleyAxis::CopleyAxis(ethercat::SoemMaster &master, int slave_index, Axis axis)
    : master_(master), slave_index_(slave_index), axis_(axis), layout_(layout_for(axis)) {}

void CopleyAxis::clear_latching_faults() {
    master_.sdo_write(slave_index_, kLatchingFaultStatusIndex + layout_.axis_object_offset, 0,
                       &kClearAllLatchingFaults, sizeof(kClearAllLatchingFaults));
}

namespace {
std::optional<std::uint32_t> checked_sdo_read_u32(const ethercat::SoemMaster &master, int slave_index,
                                                   std::uint16_t index) {
    std::uint32_t value = 0;
    int wkc = master.sdo_read(slave_index, index, 0, &value, sizeof(value));
    if (wkc <= 0) {
        return std::nullopt;
    }
    return value;
}
}  // namespace

std::optional<std::uint32_t> CopleyAxis::read_safety_circuit_status() const {
    return checked_sdo_read_u32(master_, slave_index_, kSafetyCircuitStatusIndex + layout_.axis_object_offset);
}

std::optional<std::uint32_t> CopleyAxis::read_fault_mask() const {
    return checked_sdo_read_u32(master_, slave_index_, kFaultMaskIndex + layout_.axis_object_offset);
}

std::optional<std::uint32_t> CopleyAxis::read_latching_fault_status() const {
    return checked_sdo_read_u32(master_, slave_index_, kLatchingFaultStatusIndex + layout_.axis_object_offset);
}

void CopleyAxis::update() {
    last_statusword_ = read_field<std::uint16_t>(master_, slave_index_, layout_.statusword_offset);
    const bool was_faulted = fsm_.has_fault();
    fsm_.update(last_statusword_);

    if (fsm_.has_fault() && !was_faulted) {
        // SDO/mailbox read -- only on the fault edge, not every cycle.
        int wkc = master_.sdo_read(slave_index_, cia402::kErrorCode + layout_.axis_object_offset, 0,
                                    &last_error_code_, sizeof(last_error_code_));
        last_error_code_read_failed_ = wkc <= 0;
        if (last_error_code_read_failed_) {
            last_error_code_ = 0;  // don't leave a stale value from a previous fault
        }
    } else if (!fsm_.has_fault()) {
        last_error_code_ = 0;
        last_error_code_read_failed_ = false;
    }

    last_controlword_ = fsm_.next_controlword_bits();
    write_field<std::uint16_t>(master_, slave_index_, layout_.controlword_offset, last_controlword_);

    // Safe default is zero, not "hold current command" -- forced to zero
    // whenever not actively enabling, so a stale nonzero target can never
    // be re-applied by accident on a later enable.
    last_written_velocity_counts_per_s_ = fsm_.wants_enable() ? commanded_velocity_counts_per_s_ : 0;
    write_field<std::int32_t>(master_, slave_index_, layout_.target_velocity_offset,
                               last_written_velocity_counts_per_s_);
    write_field<std::int16_t>(master_, slave_index_, layout_.torque_offset_offset, static_cast<std::int16_t>(0));

    last_velocity_actual_counts_per_s_ = read_field<std::int32_t>(master_, slave_index_, layout_.velocity_actual_offset);
    last_position_actual_counts_ = read_field<std::int32_t>(master_, slave_index_, layout_.position_actual_offset);
    last_following_error_counts_ = read_field<std::int32_t>(master_, slave_index_, layout_.following_error_offset);
    last_torque_actual_raw_ = read_field<std::int16_t>(master_, slave_index_, layout_.torque_actual_offset);
}

StateSnapshot CopleyAxis::snapshot() const {
    StateSnapshot s{};
    s.state = fsm_.state();
    s.statusword_raw = last_statusword_;
    s.controlword_raw = last_controlword_;
    s.commanded_velocity_counts_per_s = last_written_velocity_counts_per_s_;
    s.velocity_actual_counts_per_s = last_velocity_actual_counts_per_s_;
    s.position_actual_counts = last_position_actual_counts_;
    s.following_error_counts = last_following_error_counts_;
    s.torque_actual_raw = last_torque_actual_raw_;
    s.error_code = last_error_code_;
    s.has_fault = fsm_.has_fault();
    s.sto_active = last_error_code_ == kCopleyStoErrorCode;
    s.error_code_read_failed = last_error_code_read_failed_;
    return s;
}

}  // namespace copley
