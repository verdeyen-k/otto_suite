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
// 0x1600/0x1A00 + SM2/SM3 assignment + Modes of Operation := Profile
// Position) on `master` for `slave_index`. Call after master.scan(),
// before master.configure_pdos(). profile_velocity_deg_s/
// profile_accel_deg_s2/profile_decel_deg_s2 are the drive's own on-board
// trajectory-shaping limits (0x6081/0x6083/0x6084), written once via SDO;
// the defaults match this robot's steering config defaults (see
// config/robot_constants.yaml's max_steer_rate_deg_s/
// max_steer_accel_deg_s2) so tools that don't move a steering actuator in
// anger (zeroerr_state, zeroerr_move, ...) don't need to pass real values.
void configure_zeroerr_pdos(ethercat::SoemMaster &master, int slave_index, double profile_velocity_deg_s = 180.0,
                             double profile_accel_deg_s2 = 600.0, double profile_decel_deg_s2 = 600.0);

// One ZeroErr eRob axis, CiA 402 Profile Position (PP) mode.
//
// Chosen over Cyclic Synchronous Position (CSP) because CSP has no
// feedforward path this driver can reach -- the manual's CSP object set
// (Table 5-8) doesn't include a feedforward object at all, and streaming
// raw position setpoints with none produced severely underdamped/
// oscillating tracking on real hardware. PP mode instead has the DRIVE
// run its own on-board trajectory profiler (the same one the vendor's
// eTuner tooling exercises, which tracks cleanly) between host-supplied
// target updates -- see set_target_angle_deg().
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

    // On-demand SDO reads of the SM2/SM3 (0x1C32/0x1C33) DC diagnostic
    // objects (manual p.42, Table 4-8): "SM event missed" (sub-index
    // 0x0C, a counter of missed sync events -- the slave's own record of
    // whether the master is actually meeting its cyclic timing) and
    // "Synchronization error" (sub-index 0x20, a bool). Direct,
    // quantitative evidence for whether a real-time kernel would actually
    // help, instead of guessing from symptoms alone.
    [[nodiscard]] std::optional<std::uint16_t> read_sm_event_missed(bool outputs) const;
    [[nodiscard]] std::optional<bool> read_sync_error(bool outputs) const;

    // target_deg is an absolute target angle in degrees. Recorded as the
    // latest desired target; update() decides whether/when to actually
    // trigger a new Profile Position move (a target that hasn't changed
    // since the last triggered move is just held, not re-triggered every
    // cycle -- see update()'s controlword bit4/bit5 handling).
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

    // Profile Position move-triggering state (see update()). pending_ is
    // the latest requested target, in encoder counts; triggered_ is the
    // target of the last move actually kicked off with a controlword bit4
    // rising edge (nullopt until the first one). bit4_high_ mirrors what
    // was last written to controlword bit4; needs_falling_edge_first_
    // means a retarget was requested while bit4_high_ was already true, so
    // this cycle must drop it to 0 before the next cycle can raise it
    // again (bit4 is rising-edge triggered -- see kControlwordBitNewSetpoint's
    // comment in zeroerr_identity.hpp).
    std::optional<std::int32_t> pending_target_counts_;
    std::optional<std::int32_t> triggered_target_counts_;
    bool bit4_high_ = false;
    bool needs_falling_edge_first_ = false;

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
