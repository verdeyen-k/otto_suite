// ZeroErr eRob identity and PDO layout constants.
//
// Everything here was independently re-derived from the vendor manuals
// (copied into docs/zeroerr/) rather than assumed, per project direction:
//   - Vendor ID / product code: eRob CANopen/EtherCAT User Manual's
//     Identity Object section; cross-checked against the
//     alesof/ethercat_ROS2 zeroerr_driver_ros2 reference config, which
//     targets the same vendor/product and agrees.
//   - Encoder resolution: eRob Rotary Actuator User Manual V3.42, section
//     9.1 ("single-turn position feedback of 19-bit resolution is
//     0~524287") -- and section 12.1.2's speed-conversion formulas use this
//     same 524288 constant with no separate gear-ratio term, confirming
//     0x6064/0x607A are already output-shaft-referenced counts, not
//     motor-side.
//   - PDO object usage: eRob CANopen/EtherCAT User Manual, object
//     dictionary (section 8.2) and PDO communication section, which
//     explicitly states 0x1600/0x1A00 are the slots supporting arbitrary
//     mapping (as opposed to 0x1601-0x1606/0x1A01-0x1A04).
#pragma once

#include <cstdint>

namespace zeroerr {

constexpr std::uint32_t kVendorId = 0x5A65726F;
constexpr std::uint32_t kProductCode = 0x00029252;

// 2^19 -- 19-bit output-shaft absolute encoder resolution.
constexpr int kEncoderCountsPerRev = 524288;

constexpr std::uint16_t kRxPdoIndex = 0x1600;
constexpr std::uint16_t kTxPdoIndex = 0x1A00;

// Byte offsets for the extended PDO mapping this driver configures via SDO
// at connect time (see configure_zeroerr_pdos), rather than relying on the
// factory-default 3-field layout -- gives the state-dump tool real
// velocity/effort/mode-of-operation telemetry per cycle instead of needing
// extra SDO reads. NOT yet verified against real hardware; see plan
// verification step 2 (read-only bus check gates this before anything
// moves).
namespace pdo_layout {

// RxPDO 0x1600, 16 bytes total.
constexpr int kTargetPositionOffset = 0;   // 0x607A, i32
constexpr int kDigitalOutputsOffset = 4;   // 0x60FE, u32 -- bit0 = brake (0=engage, 1=release; manual sec 8.2.125)
constexpr int kControlwordOffset = 8;      // 0x6040, u16
constexpr int kTargetVelocityOffset = 10;  // 0x60FF, i32 (unused in CSP mode, mapped for completeness)
constexpr int kTargetTorqueOffset = 14;    // 0x6071, i16 (unused in CSP mode, mapped for completeness)
constexpr int kRxBytes = 16;

// TxPDO 0x1A00, 18 bytes total.
constexpr int kPositionActualOffset = 0;          // 0x6064, i32
constexpr int kVelocityActualOffset = 4;          // 0x606C, i32
constexpr int kEffortActualOffset = 8;            // 0x6077, i16 (torque actual value)
constexpr int kDigitalInputsOffset = 10;          // 0x60FD, u32
constexpr int kStatuswordOffset = 14;             // 0x6041, u16
constexpr int kModeOfOperationDisplayOffset = 16; // 0x6061, u16 on this device (manual object table lists it
                                                   // without an explicit bit width; the ROS2 reference config
                                                   // for the same vendor/product maps it as u16 -- consistent
                                                   // with word-aligned TxPDO fields elsewhere in this layout)
constexpr int kTxBytes = 18;

}  // namespace pdo_layout

constexpr std::uint32_t kDigitalOutputsBrakeReleaseBit = 1u << 0;

}  // namespace zeroerr
