#pragma once

#include <cstdint>
#include <optional>

#include "cia402/state_machine.hpp"
#include "ethercat/soem_master.hpp"

namespace zeroerr {

struct StateSnapshot {
    cia402::DriveState state;
    std::uint16_t statusword_raw;
    std::uint16_t controlword_raw;
    std::uint32_t digital_outputs_raw;  // what we last wrote (bit0 = brake release requested)
    std::uint32_t digital_inputs_raw;   // what the slave last reported
    std::int32_t position_counts;
    double position_deg;
    std::int32_t velocity_actual_counts_per_s;
    double velocity_deg_per_s;
    std::int16_t effort_actual_raw;
    std::uint16_t mode_of_operation_display;
    std::uint16_t error_code;  // 0 if no fault, else last-captured DS402 Error Code (0x603F)
    bool has_fault;
    bool sto_active;
    // True if has_fault but the 0x603F SDO read on the fault edge failed
    // (mailbox abort/timeout) -- error_code is then unknown, not "no
    // error". See the identical field/rationale on copley::StateSnapshot.
    bool error_code_read_failed;
};

double counts_to_deg(std::int32_t counts);
std::int32_t deg_to_counts(double deg);

// Registers the ZeroErr eRob's config_func (extended PDO mapping on
// 0x1600/0x1A00 + SM2/SM3 assignment + Modes of Operation := CSP) on
// `master` for `slave_index`. Call after master.scan(), before
// master.configure_pdos().
void configure_zeroerr_pdos(ethercat::SoemMaster &master, int slave_index);

// One ZeroErr eRob axis, CiA 402 Cyclic Synchronous Position (CSP) mode.
//
// The eRob has a holding brake controlled via the digital-outputs word
// (0x60FE) bit 0 (0=engaged, 1=released, per the CANopen/EtherCAT manual
// sec 8.2.125). Since a real EtherCAT master's output buffer starts
// zeroed, the brake stays engaged unless this class explicitly releases
// it -- it does so proactively as soon as enable is requested (via
// StateMachine::wants_enable(), not only once OPERATION_ENABLED is
// confirmed), and re-engages whenever enable is not being requested
// (fail-safe holding).
class ZeroErrActuator {
public:
    ZeroErrActuator(ethercat::SoemMaster &master, int slave_index);

    void enable() { fsm_.request_enable(); }
    void disable() { fsm_.request_disable(); }
    void fault_reset() { fsm_.request_fault_reset(); }

    [[nodiscard]] bool is_operational() const { return fsm_.is_operational(); }
    [[nodiscard]] bool has_fault() const { return fsm_.has_fault(); }

    // On-demand SDO read of the Error Code (0x603F), independent of the
    // CiA-402 statusword FAULT bit -- update()/snapshot() only capture
    // this reactively, on a fault *statusword* edge, so a slave stuck at
    // the EtherCAT AL layer (e.g. never leaving SAFE_OP) without ever
    // reporting a DS402-level FAULT is otherwise invisible here, even if
    // its error register is non-zero. Mailbox round trip -- call only
    // when about to display it, not every cycle. Returns std::nullopt if
    // the SDO read itself failed.
    [[nodiscard]] std::optional<std::uint16_t> read_error_code_live() const;

    // target_deg is an absolute target angle in degrees.
    void set_target_angle_deg(double target_deg);

    // Reads statusword/feedback from the bus, advances the CiA402 state
    // machine, and writes controlword + holding-brake bit. Must be called
    // once per PDO cycle, before set_target_angle_deg()/snapshot() for
    // that cycle.
    void update();

    [[nodiscard]] StateSnapshot snapshot() const;

private:
    ethercat::SoemMaster &master_;
    int slave_index_;
    cia402::StateMachine fsm_;
    std::optional<double> last_commanded_deg_;

    std::uint16_t last_statusword_ = 0;
    std::uint16_t last_controlword_ = 0;
    std::uint32_t last_digital_outputs_ = 0;
    std::uint32_t last_digital_inputs_ = 0;
    std::int32_t last_position_counts_ = 0;
    std::int32_t last_velocity_counts_per_s_ = 0;
    std::int16_t last_effort_raw_ = 0;
    std::uint16_t last_mode_of_operation_display_ = 0;
    std::uint16_t last_error_code_ = 0;
    bool last_error_code_read_failed_ = false;
};

}  // namespace zeroerr
