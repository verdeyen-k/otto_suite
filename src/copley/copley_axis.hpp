#pragma once

#include <cstdint>
#include <optional>

#include "cia402/state_machine.hpp"
#include "copley/copley_identity.hpp"
#include "ethercat/soem_master.hpp"

namespace copley {

struct StateSnapshot {
    cia402::DriveState state;
    std::uint16_t statusword_raw;
    std::uint16_t controlword_raw;
    std::int32_t commanded_velocity_counts_per_s;  // what we last wrote (0 unless actively enabling)
    std::int32_t velocity_actual_counts_per_s;
    std::int32_t position_actual_counts;
    std::int32_t following_error_counts;
    std::int16_t torque_actual_raw;
    std::uint16_t error_code;  // 0 if no fault, else last-captured DS402 Error Code (0x603F)
    bool has_fault;
    bool sto_active;
};

// Registers the Copley BE2's config_func on `master` for `slave_index`:
// maps Axis B's custom PDO (0x1601/0x1A01, mirroring Axis A's fixed
// 0x1701/0x1B00 field set via Axis B's own +0x800 objects), assigns SM2/SM3
// to carry BOTH axes' PDOs together (switching Axis A off the CSP default
// 0x1700 onto the CSV fixed 0x1701 in the process), sets Desired State :=
// 30 and Modes of Operation := CSV for both axes. Always configures both
// axes regardless of which one a given tool run will actually drive --
// this is the one combined-mapping configuration confirmed to work on real
// BE2-090-20 hardware (see copley_identity.hpp); a single-axis-only
// mapping was not tried and is not what this driver does. Call after
// master.scan(), before master.configure_pdos().
void configure_copley_pdos(ethercat::SoemMaster &master, int slave_index);

// One axis (A or B) of a Copley BE2-090-20, CiA 402 Cyclic Synchronous
// Velocity (CSV) mode.
//
// Unlike a CSP position axis, CSV's safe default target is simply zero,
// not "hold current position" -- so target velocity is forced to zero
// whenever this axis isn't actively enabling/enabled (see update()),
// rather than needing a "seed from measured value" step. This also means
// a stale nonzero command from a previous run can never be re-applied by
// accident on a later enable.
class CopleyAxis {
public:
    CopleyAxis(ethercat::SoemMaster &master, int slave_index, Axis axis);

    void enable() { fsm_.request_enable(); }
    void disable() { fsm_.request_disable(); }
    void fault_reset() { fsm_.request_fault_reset(); }

    // Copley-specific recovery path, genuinely separate from the standard
    // controlword Reset Fault bit (confirmed in the manual, p.69: writing
    // a 1 to a bit in the Latching Fault Status Register 0x2183 clears
    // that latched fault) -- call alongside fault_reset() when a fault
    // doesn't clear from the controlword pulse alone.
    void clear_latching_faults();

    // On-demand SDO read of the live Status of Safety Circuit (0x219D) --
    // deliberately NOT part of update()'s per-cycle PDO exchange, since an
    // SDO/mailbox round trip is much slower than one PDO cycle; call this
    // only when you're about to display it (e.g. at a throttled print
    // rate), not every cycle. See copley_identity.hpp for the bit meaning.
    // Returns std::nullopt if the SDO read itself failed (e.g. the object
    // isn't supported on this drive) -- distinct from a real all-zero
    // reading, which SOEM's ecx_SDOread cannot be told apart from if the
    // failure is silently ignored.
    [[nodiscard]] std::optional<std::uint32_t> read_safety_circuit_status() const;

    [[nodiscard]] bool is_operational() const { return fsm_.is_operational(); }
    [[nodiscard]] bool has_fault() const { return fsm_.has_fault(); }

    void set_target_velocity_counts_per_s(std::int32_t counts_per_s) {
        commanded_velocity_counts_per_s_ = counts_per_s;
    }

    // Reads statusword/feedback from the bus, advances the CiA402 state
    // machine, and writes controlword + target velocity (zeroed unless
    // actively enabling). Must be called once per PDO cycle, before
    // set_target_velocity_counts_per_s()/snapshot() for that cycle.
    void update();

    [[nodiscard]] StateSnapshot snapshot() const;

private:
    ethercat::SoemMaster &master_;
    int slave_index_;
    Axis axis_;
    const PdoLayout &layout_;
    cia402::StateMachine fsm_;
    std::int32_t commanded_velocity_counts_per_s_ = 0;

    std::uint16_t last_statusword_ = 0;
    std::uint16_t last_controlword_ = 0;
    std::int32_t last_written_velocity_counts_per_s_ = 0;
    std::int32_t last_velocity_actual_counts_per_s_ = 0;
    std::int32_t last_position_actual_counts_ = 0;
    std::int32_t last_following_error_counts_ = 0;
    std::int16_t last_torque_actual_raw_ = 0;
    std::uint16_t last_error_code_ = 0;
};

}  // namespace copley
