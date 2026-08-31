// Combined 8-actuator soak test that drives all 4 swerve modules through
// the kinematics module (src/kinematics/swerve_kinematics.hpp) instead of
// commanding each actuator independently -- exercises plan.md's Layer 3
// for the first time against real hardware: ChassisSpeeds -> per-module
// (angle, speed) via SwerveKinematics::to_module_states(), steering-delta
// minimization via SwerveKinematics::optimize(), unit conversion via
// config/robot_constants.yaml's wheel radius + drive encoder scale, and the
// forward direction (measured per-module state -> reconstructed chassis
// speed) via SwerveKinematics::to_chassis_speeds(), printed periodically
// as an odometry sanity check against what was actually commanded.
//
// Runs a fixed sequence of chassis motions -- forward, strafe, rotate in
// place, diagonal, stop -- each ramped in and held, exactly the shape a
// teleop/ZeroMQ command stream will eventually produce.
//
// Run with a human physically present, hand on the E-stop, wheels off the
// ground / free to spin, steering free to rotate. Defaults are
// deliberately conservative (this is the first time real hardware has ever
// been driven through the kinematics layer) -- well under the 500,000
// counts/s ceiling already proven safe in full_shake.
//
// Usage:
//   sudo ./swerve_kinematics_test --iface enp1s0 \
//     --fl steer=5,drive=2,axis=a \
//     --fr steer=7,drive=2,axis=b \
//     --rl steer=8,drive=3,axis=a \
//     --rr steer=9,drive=3,axis=b \
//     [--max-speed-mps 0.15] [--max-omega-deg-s 30] \
//     [--phase-duration-s 4] [--ramp-s 1] [--stagger-ms 100] \
//     [--config config/robot_constants.yaml]
//
// Module positions (front-left/front-right/rear-left/rear-right) are fixed
// by the config file's chassis geometry -- --fl/--fr/--rl/--rr assign
// which physical slave plays which role, all four are required.
#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <exception>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "cia402/state_machine.hpp"
#include "copley/copley_axis.hpp"
#include "copley/copley_identity.hpp"
#include "ethercat/soem_master.hpp"
#include "kinematics/swerve_kinematics.hpp"
#include "robot/robot_config.hpp"
#include "zeroerr/steer_position_controller.hpp"
#include "zeroerr/zeroerr_actuator.hpp"
#include "zeroerr/zeroerr_identity.hpp"

namespace {

std::atomic<bool> g_stop{false};
void on_sigint(int) { g_stop.store(true); }

constexpr std::size_t kFrontLeft = 0;
constexpr std::size_t kFrontRight = 1;
constexpr std::size_t kRearLeft = 2;
constexpr std::size_t kRearRight = 3;
constexpr const char *kModuleNames[4] = {"FL", "FR", "RL", "RR"};

struct ModuleTarget {
    int steer_slave = -1;
    int drive_slave = -1;
    copley::Axis drive_axis = copley::Axis::A;
};

struct Args {
    std::string iface;
    std::array<ModuleTarget, 4> modules;
    std::array<bool, 4> have_module{false, false, false, false};
    double max_speed_mps = 0.15;
    double max_omega_deg_s = 30.0;
    double phase_duration_s = 4.0;
    double ramp_s = 1.0;
    double stagger_ms = 100.0;
    std::string config_path = robot::kDefaultConfigPath;
    bool no_drive = false;
    double steer_kp = zeroerr::kDefaultSteerKp;
};

[[noreturn]] void usage_and_exit(const char *prog) {
    std::fprintf(stderr,
                 "Usage: %s --iface IFNAME --fl steer=N,drive=M,axis={a|b} --fr ... --rl ... --rr ...\n"
                 "         [--max-speed-mps 0.15] [--max-omega-deg-s 30]\n"
                 "         [--phase-duration-s 4] [--ramp-s 1] [--stagger-ms 100]\n"
                 "         [--config config/robot_constants.yaml] [--no-drive] [--steer-kp 6.0]\n"
                 "  --fl/--fr/--rl/--rr default to the robot config file's bus locations; pass a\n"
                 "  flag to override one for this run.\n"
                 "  Runs forward -> strafe -> rotate -> diagonal -> stop, each ramped in\n"
                 "  and held for phase-duration-s.\n"
                 "  --no-drive: never enable the drive axes (steering only) -- they stay in\n"
                 "  SWITCH_ON_DISABLED the whole run, so no drive torque is ever possible\n"
                 "  regardless of what a commanded speed would otherwise be.\n"
                 "  --steer-kp: proportional gain for the host-side steer position loop --\n"
                 "  see zeroerr/steer_position_controller.hpp.\n",
                 prog);
    std::exit(2);
}

const char *axis_name(copley::Axis axis) { return axis == copley::Axis::A ? "A" : "B"; }

ModuleTarget from_bus_location(const robot::ModuleBusLocation &loc) {
    return ModuleTarget{loc.steer_slave, loc.drive_slave,
                         loc.drive_axis == 'a' ? copley::Axis::A : copley::Axis::B};
}

ModuleTarget parse_module(const std::string &spec, const char *prog) {
    ModuleTarget t;
    bool have_steer = false, have_drive = false, have_axis = false;
    std::stringstream ss(spec);
    std::string kv;
    while (std::getline(ss, kv, ',')) {
        auto eq = kv.find('=');
        if (eq == std::string::npos) usage_and_exit(prog);
        std::string key = kv.substr(0, eq);
        std::string value = kv.substr(eq + 1);
        if (key == "steer") {
            t.steer_slave = std::atoi(value.c_str());
            have_steer = true;
        } else if (key == "drive") {
            t.drive_slave = std::atoi(value.c_str());
            have_drive = true;
        } else if (key == "axis") {
            if (value == "a" || value == "A") {
                t.drive_axis = copley::Axis::A;
            } else if (value == "b" || value == "B") {
                t.drive_axis = copley::Axis::B;
            } else {
                usage_and_exit(prog);
            }
            have_axis = true;
        } else {
            std::fprintf(stderr, "error: unknown module key '%s'\n", key.c_str());
            usage_and_exit(prog);
        }
    }
    if (!have_steer || !have_drive || !have_axis) usage_and_exit(prog);
    return t;
}

Args parse_args(int argc, char **argv) {
    Args args;
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        auto next = [&]() -> std::string {
            if (i + 1 >= argc) usage_and_exit(argv[0]);
            return argv[++i];
        };
        if (arg == "--iface") {
            args.iface = next();
        } else if (arg == "--fl") {
            args.modules[kFrontLeft] = parse_module(next(), argv[0]);
            args.have_module[kFrontLeft] = true;
        } else if (arg == "--fr") {
            args.modules[kFrontRight] = parse_module(next(), argv[0]);
            args.have_module[kFrontRight] = true;
        } else if (arg == "--rl") {
            args.modules[kRearLeft] = parse_module(next(), argv[0]);
            args.have_module[kRearLeft] = true;
        } else if (arg == "--rr") {
            args.modules[kRearRight] = parse_module(next(), argv[0]);
            args.have_module[kRearRight] = true;
        } else if (arg == "--max-speed-mps") {
            args.max_speed_mps = std::atof(next().c_str());
        } else if (arg == "--max-omega-deg-s") {
            args.max_omega_deg_s = std::atof(next().c_str());
        } else if (arg == "--phase-duration-s") {
            args.phase_duration_s = std::atof(next().c_str());
        } else if (arg == "--ramp-s") {
            args.ramp_s = std::atof(next().c_str());
        } else if (arg == "--stagger-ms") {
            args.stagger_ms = std::atof(next().c_str());
        } else if (arg == "--config") {
            args.config_path = next();
        } else if (arg == "--no-drive") {
            args.no_drive = true;
        } else if (arg == "--steer-kp") {
            args.steer_kp = std::atof(next().c_str());
        } else {
            usage_and_exit(argv[0]);
        }
    }
    if (args.iface.empty()) usage_and_exit(argv[0]);
    return args;
}

constexpr auto kCycle = std::chrono::microseconds(5000);
constexpr double kCycleSeconds = kCycle.count() / 1e6;

int cycles_for(double seconds) { return static_cast<int>(seconds / kCycleSeconds); }

// Direct-drive wheel <-> Copley motor encoder conversion (robot config).
// Module-agnostic overloads (no drive_invert applied) are for
// informational conversions not tied to one module's hardware, e.g. the
// max-speed printout below.
std::int32_t mps_to_counts_per_s(const robot::RobotConfig &cfg, double mps) {
    double wheel_rev_per_s = mps / (2.0 * M_PI * cfg.wheel_radius_m());
    double motor_rev_per_s = wheel_rev_per_s * cfg.drive_gear_ratio;
    return static_cast<std::int32_t>(std::lround(motor_rev_per_s * cfg.drive_encoder_counts_per_rev));
}
double counts_per_s_to_mps(const robot::RobotConfig &cfg, std::size_t module, std::int32_t counts_per_s) {
    double motor_rev_per_s = static_cast<double>(counts_per_s) / cfg.drive_encoder_counts_per_rev;
    double wheel_rev_per_s = motor_rev_per_s / cfg.drive_gear_ratio;
    double mps = wheel_rev_per_s * 2.0 * M_PI * cfg.wheel_radius_m();
    return cfg.drive_invert[module] ? -mps : mps;
}
std::int32_t mps_to_counts_per_s(const robot::RobotConfig &cfg, std::size_t module, double mps) {
    if (cfg.drive_invert[module]) mps = -mps;
    return mps_to_counts_per_s(cfg, mps);
}

kinematics::ChassisSpeeds lerp(const kinematics::ChassisSpeeds &a, const kinematics::ChassisSpeeds &b,
                                double t) {
    return kinematics::ChassisSpeeds{a.vx_mps + (b.vx_mps - a.vx_mps) * t,
                                      a.vy_mps + (b.vy_mps - a.vy_mps) * t,
                                      a.omega_rad_per_s + (b.omega_rad_per_s - a.omega_rad_per_s) * t};
}

void print_unhealthy_slaves(ethercat::SoemMaster &master) {
    int wkc = master.send_receive();
    std::fprintf(stderr, "  working counter: actual=%d expected=%d\n", wkc, master.expected_wkc());
    for (const auto &s : master.read_all_slave_states()) {
        if (s.al_state != 0x08 /* EC_STATE_OPERATIONAL */) {
            std::fprintf(stderr, "  slave [%d] name='%s' AL state=0x%02X status=0x%04X (%s)\n", s.slave_index,
                         s.name.c_str(), s.al_state, s.al_status_code, s.al_status_string.c_str());
        }
    }
}

}  // namespace

int main(int argc, char **argv) {
    Args args = parse_args(argc, argv);
    std::signal(SIGINT, on_sigint);

    robot::RobotConfig cfg;
    try {
        cfg = robot::load_robot_config(args.config_path);
    } catch (const std::exception &e) {
        std::fprintf(stderr, "error: %s\n", e.what());
        return 1;
    }
    const auto module_positions = cfg.module_positions();
    for (std::size_t j = 0; j < args.modules.size(); ++j) {
        if (!args.have_module[j]) args.modules[j] = from_bus_location(cfg.module_bus_locations[j]);
    }

    const double max_omega_rad_s = args.max_omega_deg_s * M_PI / 180.0;
    std::printf("max_speed=%.3f m/s (%d counts/s), max_omega=%.1f deg/s\n", args.max_speed_mps,
                mps_to_counts_per_s(cfg, args.max_speed_mps), args.max_omega_deg_s);

    // Fixed motion sequence: forward, strafe left, rotate CCW in place,
    // diagonal (forward+strafe combined -- the case that most exercises
    // to_module_states() actually combining vx/vy/omega together), stop.
    const std::vector<kinematics::ChassisSpeeds> kPhases = {
        {args.max_speed_mps, 0.0, 0.0},
        {0.0, args.max_speed_mps, 0.0},
        {0.0, 0.0, max_omega_rad_s},
        {args.max_speed_mps * 0.7071, args.max_speed_mps * 0.7071, 0.0},
        {0.0, 0.0, 0.0},
    };
    const char *kPhaseNames[] = {"forward", "strafe-left", "rotate-ccw", "diagonal", "stop"};
    const double total_phase_seconds = args.phase_duration_s * kPhases.size();

    ethercat::SoemMaster master(args.iface);
    int slave_count;
    try {
        slave_count = master.scan();
    } catch (const std::exception &e) {
        std::fprintf(stderr, "error: %s\n", e.what());
        return 1;
    }
    std::printf("Found %d slave(s) on '%s'.\n", slave_count, args.iface.c_str());

    for (std::size_t m = 0; m < args.modules.size(); ++m) {
        const auto &t = args.modules[m];
        std::printf("Module %s: steer slave [%d] name='%s', drive slave [%d]/%s name='%s'.\n",
                    kModuleNames[m], t.steer_slave, master.slave_name(t.steer_slave).c_str(), t.drive_slave,
                    axis_name(t.drive_axis), master.slave_name(t.drive_slave).c_str());
        zeroerr::configure_zeroerr_pdos(master, t.steer_slave);
    }
    std::vector<int> configured_drive_slaves;
    for (const auto &t : args.modules) {
        if (std::find(configured_drive_slaves.begin(), configured_drive_slaves.end(), t.drive_slave) ==
            configured_drive_slaves.end()) {
            copley::configure_copley_pdos(master, t.drive_slave);
            configured_drive_slaves.push_back(t.drive_slave);
        }
    }
    master.configure_pdos();

    if (!master.wait_for_safe_op()) {
        std::fprintf(stderr, "error: bus did not reach SAFE_OP -- aborting\n");
        print_unhealthy_slaves(master);
        return 1;
    }

    std::vector<zeroerr::ZeroErrActuator> steer_actuators;
    steer_actuators.reserve(4);
    std::vector<copley::CopleyAxis> drive_axes;
    drive_axes.reserve(4);
    for (const auto &t : args.modules) {
        steer_actuators.emplace_back(master, t.steer_slave);
        drive_axes.emplace_back(master, t.drive_slave, t.drive_axis);
    }

    auto update_all = [&]() {
        for (auto &a : steer_actuators) a.update();
        for (auto &a : drive_axes) a.update();
    };
    auto any_fault = [&]() {
        for (auto &a : steer_actuators) {
            if (a.has_fault()) return true;
        }
        for (auto &a : drive_axes) {
            if (a.has_fault()) return true;
        }
        return false;
    };
    // Same periodic-re-arm fault-clear pattern as full_shake.cpp /
    // zeroerr_move_all.cpp / copley_move_all.cpp -- see those for why a
    // single FaultReset pulse isn't reliable enough on its own.
    auto try_clear_faults = [&](const char *context) -> bool {
        if (!any_fault()) return true;
        for (std::size_t i = 0; i < steer_actuators.size(); ++i) {
            if (steer_actuators[i].has_fault()) {
                auto s = steer_actuators[i].snapshot();
                std::printf("[%s steer] Fault detected (%s, error_code=0x%04X%s). Resetting...\n",
                            kModuleNames[i], context, s.error_code, s.sto_active ? ", STO_ACTIVE" : "");
                steer_actuators[i].fault_reset();
            }
        }
        for (std::size_t i = 0; i < drive_axes.size(); ++i) {
            if (drive_axes[i].has_fault()) {
                auto s = drive_axes[i].snapshot();
                std::printf("[%s drive] Fault detected (%s, error_code=0x%04X%s). Resetting...\n",
                            kModuleNames[i], context, s.error_code, s.sto_active ? ", STO_ACTIVE" : "");
                drive_axes[i].fault_reset();
                drive_axes[i].clear_latching_faults();
            }
        }
        constexpr int kRearmEveryNCycles = 40;  // ~200ms at the 5ms cycle
        auto next_wake = std::chrono::steady_clock::now();
        for (int i = 0; i < cycles_for(1.0) && !g_stop.load(); ++i) {
            if (i % kRearmEveryNCycles == 0) {
                for (auto &a : steer_actuators) {
                    if (a.has_fault()) a.fault_reset();
                }
                for (auto &a : drive_axes) {
                    if (a.has_fault()) {
                        a.fault_reset();
                        a.clear_latching_faults();
                    }
                }
            }
            update_all();
            master.send_receive();
            next_wake += kCycle;
            std::this_thread::sleep_until(next_wake);
        }
        bool all_clear = true;
        for (std::size_t i = 0; i < steer_actuators.size(); ++i) {
            if (steer_actuators[i].has_fault()) {
                std::fprintf(stderr, "error: %s steer fault did not clear (%s) -- aborting\n", kModuleNames[i],
                             context);
                all_clear = false;
            }
        }
        for (std::size_t i = 0; i < drive_axes.size(); ++i) {
            if (drive_axes[i].has_fault()) {
                std::fprintf(stderr, "error: %s drive fault did not clear (%s) -- aborting\n", kModuleNames[i],
                             context);
                all_clear = false;
            }
        }
        return all_clear;
    };

    update_all();
    master.send_receive();

    // See full_shake.cpp: 0xA000 ("master offline") on a ZeroErr eRob is a
    // chicken-and-egg case at SAFE_OP -- non-fatal here, the checkpoint
    // right after reaching OPERATIONAL below is the one that must succeed.
    if (!try_clear_faults("at SAFE_OP")) {
        std::fprintf(stderr, "warning: fault did not clear at SAFE_OP -- trying to reach OPERATIONAL anyway\n");
    }

    if (!master.request_operational_state()) {
        std::fprintf(stderr, "error: bus did not reach OPERATIONAL state -- aborting\n");
        print_unhealthy_slaves(master);
        return 1;
    }

    update_all();
    master.send_receive();
    if (!try_clear_faults("at OPERATIONAL, before any enable command")) {
        return 1;
    }

    // Combined stagger across all 8 (steer then drive, per module in
    // FL/FR/RL/RR order) -- same current-inrush fix as full_shake.cpp.
    // --no-drive shrinks this to just the 4 steer actuators, so
    // enable_nth()/nth_operational() below never touch drive_axes at all
    // (n never reaches the drive index range) -- they're left in
    // SWITCH_ON_DISABLED for the whole run, not merely commanded zero.
    const std::size_t kTotalActuators = args.no_drive ? steer_actuators.size() : steer_actuators.size() + drive_axes.size();
    const int stagger_cycles = std::max(0, static_cast<int>(args.stagger_ms / 1000.0 / kCycleSeconds));
    auto enable_nth = [&](std::size_t n) {
        if (n < steer_actuators.size()) {
            std::printf("  [%s steer] enabling\n", kModuleNames[n]);
            steer_actuators[n].enable();
        } else {
            std::size_t d = n - steer_actuators.size();
            std::printf("  [%s drive] enabling\n", kModuleNames[d]);
            drive_axes[d].enable();
        }
    };
    auto nth_operational = [&](std::size_t n) {
        if (n < steer_actuators.size()) return steer_actuators[n].is_operational();
        return drive_axes[n - steer_actuators.size()].is_operational();
    };

    std::printf("Enabling %zu actuator(s)/axis(es), staggered %.0fms apart...\n", kTotalActuators,
                args.stagger_ms);
    std::size_t next_to_enable = 0;
    bool all_operational = false;
    auto next_wake = std::chrono::steady_clock::now();
    for (int i = 0; i < cycles_for(5.0) && !g_stop.load(); ++i) {
        if (next_to_enable < kTotalActuators && i >= static_cast<int>(next_to_enable) * stagger_cycles) {
            enable_nth(next_to_enable);
            ++next_to_enable;
        }
        update_all();
        master.send_receive();
        bool all_ok = true;
        for (std::size_t j = 0; j < kTotalActuators; ++j) {
            if (j >= next_to_enable || !nth_operational(j)) all_ok = false;
        }
        if (all_ok) {
            all_operational = true;
            break;
        }
        next_wake += kCycle;
        std::this_thread::sleep_until(next_wake);
    }
    if (!all_operational) {
        std::fprintf(stderr, "error: not all actuators/axes reached OPERATION_ENABLED within 5s -- aborting\n");
        for (std::size_t j = 0; j < steer_actuators.size(); ++j) {
            if (!steer_actuators[j].is_operational()) {
                std::fprintf(stderr, "  %s steer state=%s\n", kModuleNames[j],
                             std::string(cia402::to_string(steer_actuators[j].snapshot().state)).c_str());
            }
        }
        for (std::size_t j = 0; j < drive_axes.size(); ++j) {
            if (!drive_axes[j].is_operational()) {
                std::fprintf(stderr, "  %s drive state=%s\n", kModuleNames[j],
                             std::string(cia402::to_string(drive_axes[j].snapshot().state)).c_str());
            }
        }
        return 1;
    }

    kinematics::SwerveKinematics kinematics_solver(
        {module_positions[0], module_positions[1], module_positions[2], module_positions[3]});
    std::array<double, 4> last_commanded_angle_rad{};
    const double max_steer_rate_rad_s = cfg.max_steer_rate_deg_s * M_PI / 180.0;
    const double max_steer_accel_rad_s2 = cfg.max_steer_accel_deg_s2 * M_PI / 180.0;
    std::vector<zeroerr::SteerPositionController> steer_controllers;
    steer_controllers.reserve(4);
    for (std::size_t j = 0; j < 4; ++j) {
        steer_controllers.emplace_back(max_steer_rate_rad_s, max_steer_accel_rad_s2, args.steer_kp);
    }

    // Point the wheels at phase 0's target direction BEFORE commanding any
    // drive speed. Linearly ramping the CHASSIS vector itself from (0,0,0)
    // does NOT gradually rotate the steering angle: every point on a
    // straight line through the origin has the same direction as its
    // endpoint, so atan2() commits to the final bearing the instant the
    // magnitude leaves zero -- only speed actually ramps. The steer
    // position loop's own accel limiting (steer_controllers) handles the
    // smooth ramp-up from a standstill now; this phase just holds drive
    // speed at zero while it settles, so the main loop below starts from
    // an angle that already matches what it would compute near frac=0.
    std::array<kinematics::ModuleState, 4> phase0_desired = kinematics_solver.to_module_states(kPhases[0]);
    for (std::size_t j = 0; j < 4; ++j) {
        const double actual_wheel_angle_rad =
            cfg.raw_to_wheel_angle_deg(j, steer_actuators[j].snapshot().position_deg) * M_PI / 180.0;
        kinematics::ModuleState optimized =
            kinematics::SwerveKinematics::optimize(phase0_desired[j], actual_wheel_angle_rad);
        last_commanded_angle_rad[j] = optimized.angle_rad;
    }
    std::printf("Aligning steering to starting direction (drive speed held at zero)...\n");
    const int align_cycles = cycles_for(args.ramp_s) + static_cast<int>(3) * stagger_cycles;
    std::array<bool, 4> align_active{false, false, false, false};
    next_wake = std::chrono::steady_clock::now();
    for (int i = 0; i < align_cycles && !g_stop.load(); ++i) {
        update_all();
        for (std::size_t j = 0; j < 4; ++j) {
            if (!align_active[j] && i >= static_cast<int>(j) * stagger_cycles) {
                align_active[j] = true;
            }
            if (align_active[j]) {
                const double actual_wheel_angle_rad =
                    cfg.raw_to_wheel_angle_deg(j, steer_actuators[j].snapshot().position_deg) * M_PI / 180.0;
                const double steer_velocity_rad_s =
                    steer_controllers[j].update(last_commanded_angle_rad[j], actual_wheel_angle_rad, kCycleSeconds);
                steer_actuators[j].set_target_velocity_counts_per_s(zeroerr::deg_to_counts(
                    cfg.wheel_angular_velocity_to_raw_deg_s(j, steer_velocity_rad_s * 180.0 / M_PI)));
            }
        }
        master.send_receive();
        next_wake += kCycle;
        std::this_thread::sleep_until(next_wake);
    }

    // Separate stagger slots for steer vs. drive first-activation (steer
    // gets slots 0..3, drive gets slots 4..7) -- pairing a module's steer
    // and drive first commands into the SAME slot doubles the current
    // inrush at that instant instead of spreading it out, and was
    // confirmed on real hardware to fault the steering actuator on every
    // module but the very first (see full_shake.cpp, which keeps these two
    // groups fully separate for exactly this reason).
    std::array<bool, 4> steer_active{true, true, true, true};
    std::array<bool, 4> drive_active{false, false, false, false};

    std::printf("Running kinematics phase sequence (%.1fs/phase, %.1fs total)...\n", args.phase_duration_s,
                total_phase_seconds);

    // total_cycles extends past the nominal phase sequence by the combined
    // stagger delay, same reasoning as full_shake.cpp: the last module to
    // start being actively commanded (up to (kTotalActuators-1)*stagger_ms
    // late) still needs to see the full sequence, not a truncated tail.
    // Past the nominal sequence, the chassis target clamps to the final
    // "stop" phase rather than repeating, so late-starting modules ease
    // down to zero instead of being cut off mid-motion.
    const int nominal_cycles = cycles_for(total_phase_seconds);
    const int tail_cycles = static_cast<int>(kTotalActuators - 1) * stagger_cycles;
    const int total_cycles = nominal_cycles + tail_cycles;
    constexpr int kPrintEveryNCycles = 100;  // ~0.5s at the 5ms cycle
    next_wake = std::chrono::steady_clock::now();
    for (int i = 0; i < total_cycles && !g_stop.load(); ++i) {
        const double t = std::min(i, nominal_cycles - 1) * kCycleSeconds;
        std::size_t phase_index = std::min(static_cast<std::size_t>(t / args.phase_duration_s), kPhases.size() - 1);
        const double phase_start_t = phase_index * args.phase_duration_s;
        const double elapsed_in_phase = t - phase_start_t;
        const kinematics::ChassisSpeeds prev_target =
            phase_index == 0 ? kinematics::ChassisSpeeds{} : kPhases[phase_index - 1];
        const double frac = args.ramp_s > 0.0 ? std::min(1.0, elapsed_in_phase / args.ramp_s) : 1.0;
        const kinematics::ChassisSpeeds current = lerp(prev_target, kPhases[phase_index], frac);

        std::array<kinematics::ModuleState, 4> desired = kinematics_solver.to_module_states(current);
        kinematics::SwerveKinematics::desaturate_wheel_speeds(desired, cfg.max_wheel_speed_mps());

        for (std::size_t j = 0; j < 4; ++j) {
            steer_actuators[j].update();
            drive_axes[j].update();
            if (!steer_active[j] && i >= static_cast<int>(j) * stagger_cycles) {
                steer_active[j] = true;
            }
            if (!drive_active[j] && i >= static_cast<int>(4 + j) * stagger_cycles) {
                drive_active[j] = true;
            }
            // Flip-decide against the actuator's REAL measured angle, not
            // our own last-commanded value -- see teleop.cpp's identical
            // fix for why: with the position loop closed on the host
            // (steer_controllers), "last commanded" is just this loop's
            // setpoint and can legitimately sit ahead of where the wheel
            // physically is while still catching up. Deciding "which way
            // is shorter" from a stale assumed position instead of truth
            // is how a lagging actuator ends up spinning multiple turns
            // to catch up once it finally moves.
            const double actual_wheel_angle_rad =
                cfg.raw_to_wheel_angle_deg(j, steer_actuators[j].snapshot().position_deg) * M_PI / 180.0;
            kinematics::ModuleState optimized =
                kinematics::SwerveKinematics::optimize(desired[j], actual_wheel_angle_rad);
            if (steer_active[j]) {
                last_commanded_angle_rad[j] = optimized.angle_rad;
                const double steer_velocity_rad_s =
                    steer_controllers[j].update(last_commanded_angle_rad[j], actual_wheel_angle_rad, kCycleSeconds);
                steer_actuators[j].set_target_velocity_counts_per_s(zeroerr::deg_to_counts(
                    cfg.wheel_angular_velocity_to_raw_deg_s(j, steer_velocity_rad_s * 180.0 / M_PI)));
            }
            if (drive_active[j]) {
                drive_axes[j].set_target_velocity_counts_per_s(mps_to_counts_per_s(cfg, j, optimized.speed_mps));
            }
        }
        master.send_receive();

        if (i % kPrintEveryNCycles == 0) {
            // Reconstruct chassis speed from real feedback -- the other
            // half of the kinematics module (to_chassis_speeds), a genuine
            // odometry-style sanity check against what was commanded.
            std::array<kinematics::ModuleState, 4> measured{};
            for (std::size_t j = 0; j < 4; ++j) {
                auto steer_s = steer_actuators[j].snapshot();
                auto drive_s = drive_axes[j].snapshot();
                measured[j] = kinematics::ModuleState{
                    counts_per_s_to_mps(cfg, j, drive_s.velocity_actual_counts_per_s),
                    cfg.raw_to_wheel_angle_deg(j, steer_s.position_deg) * M_PI / 180.0};
            }
            kinematics::ChassisSpeeds odom = kinematics_solver.to_chassis_speeds(measured);
            std::printf(
                "[%6.1fs] phase=%-11s cmd(vx=%6.3f vy=%6.3f w=%6.2f) odom(vx=%6.3f vy=%6.3f w=%6.2f)\n", t,
                kPhaseNames[phase_index], current.vx_mps, current.vy_mps, current.omega_rad_per_s, odom.vx_mps,
                odom.vy_mps, odom.omega_rad_per_s);
            for (std::size_t j = 0; j < 4; ++j) {
                auto steer_s = steer_actuators[j].snapshot();
                auto drive_s = drive_axes[j].snapshot();
                std::printf("    [%s] steer=%7.2fdeg drive_cmd=%8d drive_act=%8d fault=%s\n", kModuleNames[j],
                            steer_s.position_deg, drive_s.commanded_velocity_counts_per_s,
                            drive_s.velocity_actual_counts_per_s,
                            (steer_s.has_fault || drive_s.has_fault) ? "FAULT" : "-");
            }
            std::fflush(stdout);
        }
        next_wake += kCycle;
        std::this_thread::sleep_until(next_wake);
    }

    std::printf("Ramping drives down to zero before disabling...\n");
    // Steering is CSV (velocity mode) now too -- stop commanding a
    // velocity immediately rather than coasting at whatever was last
    // commanded for the full ramp-down window.
    for (auto &a : steer_actuators) a.set_target_velocity_counts_per_s(0);
    const int ramp_down_cycles = cycles_for(args.ramp_s > 0.0 ? args.ramp_s : 1.0);
    next_wake = std::chrono::steady_clock::now();
    for (int i = 0; i < ramp_down_cycles; ++i) {
        double fraction = 1.0 - static_cast<double>(i) / ramp_down_cycles;
        for (auto &a : steer_actuators) a.update();
        for (std::size_t j = 0; j < drive_axes.size(); ++j) {
            drive_axes[j].update();
            drive_axes[j].set_target_velocity_counts_per_s(
                static_cast<std::int32_t>(drive_axes[j].snapshot().commanded_velocity_counts_per_s * fraction));
        }
        master.send_receive();
        next_wake += kCycle;
        std::this_thread::sleep_until(next_wake);
    }

    std::printf("Disabling and closing bus.\n");
    for (auto &a : steer_actuators) a.disable();
    for (auto &a : drive_axes) a.disable();
    next_wake = std::chrono::steady_clock::now();
    for (int i = 0; i < cycles_for(1.0); ++i) {
        update_all();
        master.send_receive();
        next_wake += kCycle;
        std::this_thread::sleep_until(next_wake);
    }
    master.close();
    return 0;
}
