// Copley BE2-090-20 (Accelnet Plus dual-axis) identity and PDO layout
// constants, CSV (Cyclic Synchronous Velocity) mode.
//
// A physical BE2-090-20 is ONE EtherCAT slave exposing TWO CiA-402 axes
// ("Axis A" and "Axis B"), not two slaves -- independently re-derived from
// the CANopen & EtherCAT Programmer's Manual (16-01195 Rev 06):
//   - p.18: standard DS402 objects (0x6000-0x67FF range) are offset by
//     +0x800 per axis -- Axis A = 0x6000-0x67FF, Axis B = 0x6800-0x6FFF.
//     ("the difference between the addresses will not be affected by the
//     number of axes in a drive... the object addresses for each axis
//     will increment by 0x800")
//   - p.26-27, p.34-35: RxPDO 0x1700 (fixed, CSP) / 0x1701 (fixed, CSV) and
//     TxPDO 0x1B00 (fixed) are Axis A's, not remappable. 0x1600-0x1603 /
//     0x1A00-0x1A03 are explicitly documented as non-fixed, user-mappable
//     slots -- used here to build Axis B's own PDO out of its +0x800
//     objects (there's no separate fixed "Axis B" PDO; the ESI only
//     defines fixed PDOs for Axis A).
//   - p.42: 0x1C12/0x1C13 (SM2/SM3 PDO assignment) support 0-4
//     simultaneously assigned PDOs -- so assigning both Axis A's fixed PDO
//     and Axis B's custom one into the same sync manager is a documented
//     capability, not an undocumented trick.
//   - p.35: RxPDO 0x1701 fixed content: Controlword(16b), Target
//     velocity(32b), Torque offset(16b) = 8 bytes.
//   - p.41: TxPDO 0x1B00 fixed content: Statusword(16b), Actual
//     position(32b), Position (following) error(32b), Actual
//     velocity(32b), Torque actual(16b) = 16 bytes.
//   - p.64-65: Desired State (0x2300) := 30 ("CANopen interface controls
//     amplifier") is what the manual says to program for CANopen/EtherCAT
//     control; p.64: Mode of Operation (0x6060) CSV = 9.
//   - p.62: Error Code (0x603F) value 0x5440 = "STO Fault".
//   - p.69: Latching Fault Status Register (0x2183) -- write a 1 to a bit
//     to clear it; a genuinely separate mechanism from the standard
//     controlword Reset Fault bit (0x6040 bit 7).
//
// Vendor ID / product code (0x000000AB / 0x000010C0) are NOT from this
// manual -- Copley's generic programmer's manual doesn't list per-model
// codes, only a separate parts catalog. These values come from a live
// EtherCAT bus scan of this exact BE2-090-20 hardware recorded in the
// earlier Python project's bring-up notes; treat as a strong lead, not
// manual-confirmed, until reconfirmed by our own bus scan.
#pragma once

#include <cstdint>
#include <vector>

#include "ethercat/soem_master.hpp"

namespace copley {

constexpr std::uint32_t kVendorId = 0x000000AB;
constexpr std::uint32_t kProductCode = 0x000010C0;

enum class Axis { A, B };

// Axis A's fixed CSV PDOs (not remappable). Axis B has no fixed PDO of its
// own -- it's built from the non-fixed 0x1601/0x1A01 slots, see
// configure_copley_pdos.
constexpr std::uint16_t kAxisAFixedRxPdoIndex = 0x1701;
constexpr std::uint16_t kAxisAFixedTxPdoIndex = 0x1B00;
constexpr std::uint16_t kAxisBRxPdoIndex = 0x1601;
constexpr std::uint16_t kAxisBTxPdoIndex = 0x1A01;

constexpr std::uint16_t kSm2RxAssignmentIndex = 0x1C12;
constexpr std::uint16_t kSm3TxAssignmentIndex = 0x1C13;

constexpr std::uint16_t kDesiredStateIndex = 0x2300;
constexpr std::uint16_t kDesiredStateCanOpenControlsAmplifier = 30;

constexpr std::uint16_t kLatchingFaultStatusIndex = 0x2183;
constexpr std::uint32_t kClearAllLatchingFaults = 0xFFFFFFFF;

constexpr std::uint16_t kCopleyStoErrorCode = 0x5440;

// Status of Safety Circuit (0x219D) -- a LIVE, continuous readout of the
// physical STO input, independent of whether the axis has ever been
// enabled (unlike the DS402 statusword FAULT bit / Error Code, which are
// populated on a fault edge and may only latch once an enable is actually
// attempted). Confirmed from the manual, p.70: "Set when safety input 0 is
// preventing the drive from enabling" -- bit 0 is what to watch when
// toggling the E-stop with the axis never enabled.
constexpr std::uint16_t kSafetyCircuitStatusIndex = 0x219D;
constexpr std::uint32_t kSafetyCircuitInput0Blocking = 1u << 0;

constexpr std::uint16_t axis_object_offset(Axis axis) { return axis == Axis::A ? 0x0000 : 0x0800; }

// Byte offsets within the combined Rx/Tx process image this driver
// configures (Axis A's fixed PDO followed immediately by Axis B's custom
// one, mirroring the same field set) -- see configure_copley_pdos.
struct PdoLayout {
    int controlword_offset;
    int target_velocity_offset;
    int torque_offset_offset;
    int statusword_offset;
    int position_actual_offset;
    int following_error_offset;
    int velocity_actual_offset;
    int torque_actual_offset;
    std::uint16_t axis_object_offset;  // for SDO addressing (Mode of Operation, Error Code, ...)
};

// Rx: 16 bytes total (Axis A 0-7, Axis B 8-15). Tx: 32 bytes total (Axis A
// 0-15, Axis B 16-31).
constexpr PdoLayout kAxisALayout{
    /*controlword_offset=*/0,        /*target_velocity_offset=*/2,     /*torque_offset_offset=*/6,
    /*statusword_offset=*/0,         /*position_actual_offset=*/2,     /*following_error_offset=*/6,
    /*velocity_actual_offset=*/10,   /*torque_actual_offset=*/14,      /*axis_object_offset=*/0x0000,
};
constexpr PdoLayout kAxisBLayout{
    /*controlword_offset=*/8,        /*target_velocity_offset=*/10,    /*torque_offset_offset=*/14,
    /*statusword_offset=*/16,        /*position_actual_offset=*/18,    /*following_error_offset=*/22,
    /*velocity_actual_offset=*/26,   /*torque_actual_offset=*/30,      /*axis_object_offset=*/0x0800,
};

constexpr const PdoLayout &layout_for(Axis axis) { return axis == Axis::A ? kAxisALayout : kAxisBLayout; }

}  // namespace copley
