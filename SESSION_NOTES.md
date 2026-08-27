# Session notes (handoff for next session)

## Where things stand

Layer 3 (kinematics) and Layer 4 (ZeroMQ bridge) from `plan.md` are now
built and wired into a live teleop path. **Not yet confirmed working
end-to-end on real hardware** — the last fix (steering acceleration
limiting) has been built and deployed but not yet driven with a joystick.

**Next action**: run `teleop` on ottoman-control, then `teleop_joystick.py`
on Windows, and actually drive it. If steering still faults, get the
`error_code` from the log (teleop now prints it) before changing anything.

## What was built this session

- `src/robot/robot_constants.hpp` — chassis geometry (475mm track width,
  625mm wheelbase, 170mm wheel diameter) and drivetrain facts (524288
  counts/rev Copley encoder, 1:1 direct drive — both confirmed by you, not
  derived from a manual).
- `src/kinematics/swerve_kinematics.{hpp,cpp}` — `ChassisSpeeds` <->
  `ModuleState`, ported from WPILib's math (not the library). Verified
  hardware-free via `kinematics_check` (builds/runs on both the dev
  sandbox and ottoman-control).
- `tools/swerve_kinematics_test.cpp` — first real-hardware exercise of the
  kinematics module (fixed phase sequence: forward/strafe/rotate/diagonal/
  stop). Needed two real-hardware fixes before it worked:
  1. Steer+drive first-command inrush paired into the same stagger slot
     (fixed: separate slots, steer 0-3 then drive 4-7, matching
     `full_shake.cpp`).
  2. Ramping the chassis vector from `(0,0,0)` doesn't gradually rotate
     steering angle — direction commits instantly, only magnitude ramps.
     Fixed with an explicit pre-loop "point wheels at phase-0 direction
     before driving" step.
- `src/bridge/` — ZeroMQ command/telemetry link (`ChassisLink`, thin
  wrapper over libzmq's C API, no cppzmq). **Do not name a namespace
  `link`** — collides with POSIX `link()` via `<unistd.h>`, pulled in
  transitively through `<zmq.h>`. Named `bridge` instead. Verified
  hardware-free via `chassis_link_check`.
- `tools/teleop_joystick.py` — Python joystick -> ChassisSpeeds publisher.
  Requires `libzmq3-dev` (C++ side) and `python3-zmq`/`python3-pygame` (or
  a venv if those apt packages don't exist) on ottoman-control, and a
  matching Python + pygame + pyzmq on whatever machine has the physical
  controller. Confirmed: an Xbox Series X Controller over Bluetooth on
  Windows is seen fine by pygame once a real Python (not the Store alias
  stub) is installed — used `winget install Python.Python.3.12`.
  **Axis mapping (0=left X, 1=left Y, 3=right X) is the standard
  XInput-via-SDL layout and matches this controller's reported 6 axes,
  but was never confirmed by actually moving the stick and watching
  the robot respond correctly** — verify this first if motion direction
  ever looks wrong.
- `tools/teleop.cpp` — live driver reading `ChassisSpeeds` from
  `ChassisLink` each cycle instead of a fixed sequence. Watchdog: hold
  last command (150ms) -> decay to zero -> disable (1000ms), re-enabling
  (staggered) on a fresh command. Two real-hardware bugs found and fixed
  here so far:
  1. The onset stagger only fired once at startup; a live joystick can go
     rest->moving->rest->moving repeatedly, and every one of those onsets
     needs its own stagger. Fixed: `wake_cycle` now resets on every
     zero->nonzero transition of the target.
  2. **`0x8400` "Velocity Error Exceeds the Limit Value"** — the steering
     rate limiter only clamped position delta per cycle (a velocity
     clamp), not the *acceleration* to reach that velocity, so the
     implied angular velocity could step from 0 to max in one 5ms cycle.
     The eRob manual (Table 12-1) requires >=0.3s to ramp to max angular
     velocity. Fixed by tracking `last_commanded_angular_velocity_rad_s`
     per module and acceleration-limiting *that* (new
     `--max-steer-accel-deg-s2`, default 600 deg/s^2), then integrating
     velocity into position, instead of just capping the position delta.
     **This fix has not yet been driven on hardware.**
  Also raised the "is the target actually nonzero" threshold from 1e-4 to
  1% of max_speed/max_omega — a real joystick's deadzone rescaling leaks
  small values (~0.003-0.004) right at its edge even at rest, which was
  quietly re-arming (and defeating) the stagger before real motion
  commands ever arrived.

## Useful facts for next time

- Module/slave mapping used in recent test commands: FL steer=5 drive=2/A,
  FR steer=7 drive=2/B, RL steer=8 drive=3/A, RR steer=9 drive=3/B. This
  came from prior `full_shake` runs, not verified against
  `docs/ecatbustopo.md`'s physical topology notes — those two have
  disagreed before (topology doc says slave 6, live testing showed slave
  7). If slave numbers ever seem off, re-scan (`slaveinfo` or any tool's
  own scan output) rather than trusting either document blindly.
- Windows machine reaches ottoman-control over Tailscale at ~340ms
  round-trip (relayed through DERP, not a direct connection) — teleop may
  feel laggy; that's why, not a bug.
- Neither the dev sandbox nor ottoman-control has passwordless `sudo` —
  any new system package needs you to run the install command yourself.
- Local syntax/build verification trick used this session: `apt-get
  download <pkg>` + `dpkg-deb -x` gets headers/libs without root, for fast
  local iteration before the real build on ottoman-control. Used for both
  `libzmq3-dev`/`libzmq5` and `poppler-utils` (for reading the ZeroErr PDF
  manuals' text directly when tracking down `0x8400`).
- Run commands:
  ```
  # on ottoman-control
  sudo ./build/teleop --iface enp1s0 \
    --fl steer=5,drive=2,axis=a --fr steer=7,drive=2,axis=b \
    --rl steer=8,drive=3,axis=a --rr steer=9,drive=3,axis=b

  # on Windows (or wherever the joystick is)
  C:\Users\KrisVerdeyen\AppData\Local\Programs\Python\Python312\python.exe \
    \\wsl.localhost\Ubuntu\home\verdeyen\Software\otto_suite\tools\teleop_joystick.py \
    --host ottoman-control
  ```

## Open items / not yet done

- Confirm the `0x8400` fix actually works by driving it live.
- Confirm joystick axis-to-motion direction mapping is actually correct
  (forward on the stick = forward on the robot, etc.) once it's safe to
  drive.
- `SwerveKinematics::to_chassis_speeds()` (odometry reconstruction) has
  only been checked against itself (round-trip in `kinematics_check`) and
  against live feedback for internal consistency in `swerve_kinematics_test`
  — never checked against independent ground truth (e.g. actual measured
  robot displacement).
- No scripted Python test-sequencing layer yet (`plan.md`'s "Test
  sequencing design" open thread) — `teleop_joystick.py` is manual/live
  only, not an automated test.
- No closed-loop swerve control (`plan.md`'s other open thread) — this is
  all open-loop command -> actuator, no chassis-level feedback control.
