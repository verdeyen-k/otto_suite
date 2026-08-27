# EtherCAT Swerve Drive Base — Plan-Mode Build Document

Purpose: hand this to Claude Code as the shared plan for building this project step by step, bottom-up, with each layer independently buildable and testable. This replaces an earlier top-down attempt that stalled out. Prefer existing open source building blocks over writing things from scratch where they exist.

## Hardware Context

- Main drive (wheels): Two Copley Controls Accelnet Plus BE2 dual-axis EtherCAT drives (4 axes total, all wired to drive wheels), driving ZL Tech hub motors. EtherCAT slave, CANopen over EtherCAT (CoE), CiA/DSP-402 profile. Controlled in velocity mode.
- Steering: Four ZeroErr eRob integrated actuators. EtherCAT (CoE) slaves, CiA-402 profile. Controlled in position mode.
- Both drive types are already selected, wired, and confirmed working at the hardware level.

## Safety

- A wireless e-stop is wired into the Safe Torque Off (STO) inputs on all drives. This has been tested — the drives physically cannot run while STO is engaged.
- Software provisions needed: detect STO/fault status on startup, and provide a clean, explicit way to clear/reset that state (as part of the CiA 402 fault-reset transition) before attempting to enable operation. Do not attempt to build a separate software safety interlock — the hardware STO is the safety layer; software just needs to work correctly with it.

## Key Reference: Open Source Building Blocks Found

1. ICube-Robotics ethercat_driver_ros2 (GitHub) — has a working ZeroErr eRob EtherCAT slave config: vendor ID, product ID, and RxPDO/TxPDO mappings (target position, control word, status word). Useful as a direct reference for the ZeroErr slave config even though we are NOT adopting ROS 2. Also has documentation specifically on CiA 402 drive state transitions.
2. CopleyControlsOfficial GitHub org — official examples for interfacing with Copley drives (note: their CML library is proprietary/paid; look for what's usable without it, e.g. raw SOEM + CoE approach).
3. SOEM (Simple Open EtherCAT Master), OpenEtherCATsociety/SOEM on GitHub — the EtherCAT master library to build on. Includes a bundled slaveinfo diagnostic tool for scanning the bus and confirming slaves are detected (already used before, don't need to rebuild).
4. kubabuda/ecat_servo (GitHub) — open source C implementation of a reusable CiA 402 state machine function (cia402_state_machine()), built around SOEM/SOES. Good reference/starting point for a proper state machine abstraction rather than hardcoding the happy path.
5. canopen-python/canopen discussion #597 — clean reference for the exact CiA 402 controlword/state sequence needed: SWITCH ON DISABLED → READY TO SWITCH ON → SWITCHED ON → OPERATION ENABLED, plus setting mode of operation (velocity vs. position) and target value via PDO.
6. WPILib SwerveDriveKinematics (C++, part of wpilibsuite on GitHub/frc-docs) — reference implementation for swerve kinematics math. Do NOT pull in full WPILib (too much unneeded bulk: sim framework, NetworkTables, command-based framework, its own units library, tied to roboRIO toolchain). Port/reference just the math.

## Architecture (Bottom-Up Layers)

### Layer 1: Drive Hardware (EtherCAT Slaves)
Already working. No build work here beyond slave configuration.

### Layer 2: Real-Time EtherCAT Master & Drivers
- Language: C or C++ (real-time/low-jitter requirements rule out Python for this layer).
- Master library: SOEM.
- Do not build ad hoc, hardcoded drivers. Research and design a proper, general CiA 402 state machine abstraction (fault reset, shutdown, switch on, enable operation, fault reaction, etc.) — see kubabuda/ecat_servo and the canopen-python reference above for the state transitions and structure. Build this as reusable capability, not just what's needed for the two immediate test cases.
- Must correctly handle the Safe Torque Off (STO) / fault status as part of this state machine (see Safety section above).
- On top of the general CiA 402 abstraction, implement two concrete drivers:
  - Copley axis driver — velocity mode. Needs to address a specific drive index AND axis number (0 or 1) since each Copley unit is dual-axis. Interface should support setting target velocity and reading actual velocity per (drive index, axis) pair.
  - ZeroErr actuator driver — position mode. Interface should support setting target angle/position and reading actual position.
- Both drivers sit on the shared CiA 402 state machine, not separate ad hoc implementations of the state handling.

### Layer 3: Swerve Kinematics Module
Custom lean C++ implementation (not full WPILib), covering:
1. Module geometry definition — x/y offset of each of the 4 steering modules relative to chassis center.
2. Forward kinematics — chassis velocity (forward, strafe, rotation) → per-module wheel speed + steering angle.
3. Inverse kinematics — per-module measured speed + angle → reconstructed chassis velocity (for odometry).
4. Wheel speed normalization — if any module's commanded speed exceeds max, scale all four proportionally.
5. Angle optimization — if the shortest rotation to target or its 180-degree opposite exceeds 90 degrees, flip wheel speed sign and target angle minus 180 instead (avoids sweeping more than 90 degrees per command).

### Layer 4: C++ to Python Interface
- ZeroMQ, chosen for mature Python bindings and fit for command/telemetry (vs. shared-memory ring buffers for lowest latency, or gRPC for strongly-typed but heavier setup).
- C++ real-time core owns the EtherCAT loop, CiA 402 drivers, and kinematics.
- Python layer handles test sequencing, orchestration, and CLI tooling.

## First Milestone (Build This First)

Goal: prove the driver layer works before anything else is built on top.

1. Build the general CiA 402 state machine abstraction (see Layer 2 research above), including STO/fault clear handling.
2. Build the Copley axis driver (velocity mode) on top of it.
3. Build the ZeroErr actuator driver (position mode) on top of it.
4. Write two small standalone test programs (modeled on the style of SOEM's own examples, e.g. slaveinfo):
   - Copley test: command a specific (drive index, axis) to a target RPM for a set duration, confirm it spins and stops.
   - ZeroErr test: command a target angle, confirm it moves there and holds.
5. Test one axis/actuator at a time first. Only after both individual drivers are proven, extend to running multiple axes/actuators together (eventually all 4 Copley axes across 2 drives, and all 4 ZeroErr actuators).

## Open Threads (Not Yet Designed — Later Steps)

- Test sequencing design (Python layer specifics)
- Exact SOEM slave configuration details for both drive types (PDO mapping specifics, sync manager config)
- Integration of kinematics layer with the drivers (full closed-loop swerve control)

