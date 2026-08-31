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
    std::int32_t commanded_velocity_counts_per_s;  // what we last wrote (0 unless actively enabling)
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
// 0x1600/0x1A00 + SM2/SM3 assignment + Modes of Operation := Cyclic
// Synchronous Velocity) on `master` for `slave_index`. Call after
// master.scan(), before master.configure_pdos().
void configure_zeroerr_pdos(ethercat::SoemMaster &master, int slave_index);

// One ZeroErr eRob axis, CiA 402 Cyclic Synchronous Velocity (CSV) mode.
//
// Previously ran Profile Position (PP) mode, letting the drive's own
// on-board trajectory profiler handle the steering position loop
// (chosen over Cyclic Synchronous Position after CSP's lack of a
// feedforward path produced underdamped/oscillating tracking). PP mode
// itself turned out to have a real-hardware failure mode that resisted
// diagnosis: the CiA-402 New Set-point/Set-point Acknowledge handshake
// would complete cleanly -- controlword bit4 raised, statusword bit12
// acknowledged within milliseconds, every time -- while the drive
// applied zero corrective torque and the physical position stayed
// frozen for seconds at a stretch, with no fault raised. Hardware,
// wiring, Control Source, a holding brake, dropped/corrupted EtherCAT
// frames (working counter stayed at expected_wkc() throughout), DC sync
// health, and Change Set Immediately were all ruled out in turn.
//
// CSV mode sidesteps the whole PP handshake and its opaque internal
// trajectory generator: the position loop is closed here, on the host
// (see the P-controller in teleop.cpp/swerve_kinematics_test.cpp), and
// this class just streams a target velocity every cycle -- the same
// well-tested pattern already used for the drive/wheel axes (see
// copley::CopleyAxis), which never showed any version of this problem.
//
// The eRob has a holding brake controlled via the digital-outputs word
// (0x60FE) bit 0 (0=engaged, 1=released, per the CANopen/EtherCAT manual
// sec 8.2.125). Since a real EtherCAT master's output buffer starts
// zeroed, the brake stays engaged unless this class explicitly releases
// it -- it does so proactively as soon as enable is requested (via
// StateMachine::wants_enable(), not only once OPERATION_ENABLED is
// confirmed), and re-engages whenever enable is not being requested
// (fail-safe holding). Target velocity is likewise forced to zero
// whenever not actively enabling/enabled (same reasoning as CopleyAxis),
// so a stale nonzero command from a previous run can never be re-applied
// by accident on a later enable.
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

    // On-demand SDO reads of the SM2/SM3 (0x1C32/0x1C33) DC diagnostic
    // objects (manual p.42, Table 4-8): "SM event missed" (sub-index
    // 0x0C, a counter of missed sync events -- the slave's own record of
    // whether the master is actually meeting its cyclic timing) and
    // "Synchronization error" (sub-index 0x20, a bool). Direct,
    // quantitative evidence for whether a real-time kernel would actually
    // help, instead of guessing from symptoms alone.
    [[nodiscard]] std::optional<std::uint16_t> read_sm_event_missed(bool outputs) const;
    [[nodiscard]] std::optional<bool> read_sync_error(bool outputs) const;

    void set_target_velocity_counts_per_s(std::int32_t counts_per_s) {
        commanded_velocity_counts_per_s_ = counts_per_s;
    }

    // Reads statusword/feedback from the bus, advances the CiA402 state
    // machine, and writes controlword + target velocity (zeroed unless
    // actively enabling) + holding-brake bit. Must be called once per PDO
    // cycle, before set_target_velocity_counts_per_s()/snapshot() for
    // that cycle.
    void update();

    [[nodiscard]] StateSnapshot snapshot() const;

private:
    ethercat::SoemMaster &master_;
    int slave_index_;
    cia402::StateMachine fsm_;
    std::int32_t commanded_velocity_counts_per_s_ = 0;

    std::uint16_t last_statusword_ = 0;
    std::uint16_t last_controlword_ = 0;
    std::uint32_t last_digital_outputs_ = 0;
    std::uint32_t last_digital_inputs_ = 0;
    std::int32_t last_written_velocity_counts_per_s_ = 0;
    std::int32_t last_position_counts_ = 0;
    std::int32_t last_velocity_counts_per_s_ = 0;
    std::int16_t last_effort_raw_ = 0;
    std::uint16_t last_mode_of_operation_display_ = 0;
    std::uint16_t last_error_code_ = 0;
    bool last_error_code_read_failed_ = false;
};

}  // namespace zeroerr
