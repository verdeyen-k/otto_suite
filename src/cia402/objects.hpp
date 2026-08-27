// CiA 402 (DS402) object dictionary indices and mode constants standardized
// by the DS402 profile itself, independent of vendor.
#pragma once

#include <cstdint>

namespace cia402 {

constexpr std::uint16_t kControlword = 0x6040;
constexpr std::uint16_t kStatusword = 0x6041;
constexpr std::uint16_t kModesOfOperation = 0x6060;
constexpr std::uint16_t kModesOfOperationDisplay = 0x6061;
constexpr std::uint16_t kErrorCode = 0x603F;

enum class ModeOfOperation : std::int8_t {
    ProfilePosition = 1,
    ProfileVelocity = 3,
    ProfileTorque = 4,
    Homing = 6,
    CyclicSyncPosition = 8,  // CSP
    CyclicSyncVelocity = 9,  // CSV
};

// Standard controlword bit patterns for each named DS402 transition.
enum class ControlwordCommand : std::uint16_t {
    Shutdown = 0x06,         // SWITCH_ON_DISABLED -> READY_TO_SWITCH_ON
    SwitchOn = 0x07,         // READY_TO_SWITCH_ON -> SWITCHED_ON
    EnableOperation = 0x0F,  // SWITCHED_ON -> OPERATION_ENABLED
    DisableVoltage = 0x00,   // any -> SWITCH_ON_DISABLED
    QuickStop = 0x02,        // any active state -> QUICK_STOP_ACTIVE
    FaultReset = 0x80,       // rising edge, FAULT -> SWITCH_ON_DISABLED
};

// Statusword bit masks (standard DS402 decode table, bits 0,1,2,3,5,6).
constexpr std::uint16_t kStatuswordMaskReadyToSwitchOn = 0x01;
constexpr std::uint16_t kStatuswordMaskSwitchedOn = 0x02;
constexpr std::uint16_t kStatuswordMaskOperationEnabled = 0x04;
constexpr std::uint16_t kStatuswordMaskFault = 0x08;
constexpr std::uint16_t kStatuswordMaskVoltageEnabled = 0x10;
constexpr std::uint16_t kStatuswordMaskQuickStop = 0x20;
constexpr std::uint16_t kStatuswordMaskSwitchOnDisabled = 0x40;
constexpr std::uint16_t kStatuswordMaskWarning = 0x80;

// Manufacturer-specific ERROR_CODE (0x603F) value that means the hardware
// Safe Torque Off input has tripped. Confirmed directly from the ZeroErr
// eRob CANopen/EtherCAT User Manual, section 7.2.23 ("STO function is
// activated").
constexpr std::uint16_t kZeroErrStoErrorCode = 0xF005;

}  // namespace cia402
