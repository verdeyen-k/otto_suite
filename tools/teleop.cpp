// Live teleop driver: brings up all 4 swerve modules exactly like
// swerve_kinematics_test.cpp, then instead of a fixed phase sequence,
// drives them from ChassisSpeeds commands received over the ZeroMQ bridge
// (src/bridge/), normally from tools/web_teleop/bridge.py (a webpage
// served from this machine, controller read via the browser's Gamepad
// API -- see that directory) or, for a no-browser CLI fallback,
// tools/teleop_joystick.py.
//
// Safety design, in order of how a live command feed can fail:
//
// 1. Per-module rate limiting (--max-steer-rate-deg-s, --max-accel-mps2)
//    is the general fix for the "instant angle jump" bug found in
//    swerve_kinematics_test.cpp: rather than special-casing the
//    zero-vector startup case, EVERY module's commanded angle and speed
//    are slew-rate-limited every cycle, every time, regardless of why the
//    desired value changed (startup, a joystick snap, a stale-link decay
//    toward zero). This also means no separate "point wheels before
//    driving" pre-phase is needed here -- the limiter handles it uniformly.
//
// 2. Stale-link watchdog: age of the last received command determines
//    behavior -- held (< --hold-ms), decayed toward zero via the same
//    rate limiters (--hold-ms to --disable-ms), or actuators actively
//    disabled (>= --disable-ms). A fresh command from DISABLED re-enables
//    (staggered, same as startup) before resuming.
//
// 3. Received commands are hard-clamped to --max-speed-mps/--max-omega-deg-s
//    on the C++ side too, not just trusted from whatever sent them.
//
// Run with a human physically present, hand on the E-stop, wheels off the
// ground / free to spin, steering free to rotate.
//
// Usage:
//   sudo ./teleop --iface enp1s0 \
//     --fl steer=5,drive=2,axis=a --fr steer=7,drive=2,axis=b \
//     --rl steer=8,drive=3,axis=a --rr steer=9,drive=3,axis=b \
//     [--command-port 5555] [--telemetry-port 5556] \
//     [--max-speed-mps 0.15] [--max-omega-deg-s 30] \
//     [--max-steer-rate-deg-s 180] [--max-steer-accel-deg-s2 600] [--max-accel-mps2 0.3] \
//     [--hold-ms 150] [--disable-ms 1000] [--stagger-ms 100]
//
// Then, on this same machine:
//   python3 tools/web_teleop/bridge.py
// and open http://<this machine's address>:8080/ from a browser on any
// machine that can reach it, with the Xbox controller attached there.
// (Or, without a browser: python3 tools/teleop_joystick.py --host <this machine's address>)
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
#include <fstream>
#include <limits>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "bridge/chassis_link.hpp"
#include "cia402/state_machine.hpp"
#include "copley/copley_axis.hpp"
#include "copley/copley_identity.hpp"
#include "ethercat/soem_master.hpp"
#include "kinematics/swerve_kinematics.hpp"
#include "robot/robot_config.hpp"
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
    int command_port = bridge::kCommandPort;
    int telemetry_port = bridge::kTelemetryPort;
    int control_port = bridge::kControlPort;
    // NaN means "not given on the command line" -- filled in from the
    // robot config file after parsing, unless overridden here.
    double max_speed_mps = std::numeric_limits<double>::quiet_NaN();
    double max_omega_deg_s = std::numeric_limits<double>::quiet_NaN();
    double max_steer_rate_deg_s = std::numeric_limits<double>::quiet_NaN();
    double max_steer_accel_deg_s2 = std::numeric_limits<double>::quiet_NaN();
    double max_accel_mps2 = std::numeric_limits<double>::quiet_NaN();
    double max_wheel_speed_rpm = std::numeric_limits<double>::quiet_NaN();
    double hold_ms = 150.0;
    double disable_ms = 1000.0;
    double stagger_ms = 100.0;
    std::string config_path = robot::kDefaultConfigPath;
    std::string log_csv_path;
};

[[noreturn]] void usage_and_exit(const char *prog) {
    std::fprintf(stderr,
                 "Usage: %s --iface IFNAME --fl steer=N,drive=M,axis={a|b} --fr ... --rl ... --rr ...\n"
                 "         [--command-port 5555] [--telemetry-port 5556] [--control-port 5557]\n"
                 "         [--max-speed-mps 0.15] [--max-omega-deg-s 30]\n"
                 "         [--max-steer-rate-deg-s 180] [--max-steer-accel-deg-s2 600] [--max-accel-mps2 0.3]\n"
                 "         [--max-wheel-speed-rpm 57] [--hold-ms 150] [--disable-ms 1000] [--stagger-ms 100]\n"
                 "         [--config config/robot_constants.yaml] [--log-csv PATH]\n"
                 "  max-speed-mps/max-omega-deg-s/max-steer-rate-deg-s/max-steer-accel-deg-s2/max-accel-mps2/\n"
                 "  max-wheel-speed-rpm/--fl/--fr/--rl/--rr all default to the robot config file's values;\n"
                 "  pass a flag to override it for one run (e.g. after a re-scan shows different slaves).\n"
                 "  --log-csv: write one row per 5ms cycle (chassis command, per-module commanded vs.\n"
                 "  actual raw steer angle, controlword bit4/statusword bit12, torque/velocity feedback)\n"
                 "  for offline comparison of joystick input against actual actuator response.\n",
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
        } else if (arg == "--command-port") {
            args.command_port = std::atoi(next().c_str());
        } else if (arg == "--telemetry-port") {
            args.telemetry_port = std::atoi(next().c_str());
        } else if (arg == "--control-port") {
            args.control_port = std::atoi(next().c_str());
        } else if (arg == "--max-speed-mps") {
            args.max_speed_mps = std::atof(next().c_str());
        } else if (arg == "--max-omega-deg-s") {
            args.max_omega_deg_s = std::atof(next().c_str());
        } else if (arg == "--max-steer-rate-deg-s") {
            args.max_steer_rate_deg_s = std::atof(next().c_str());
        } else if (arg == "--max-steer-accel-deg-s2") {
            args.max_steer_accel_deg_s2 = std::atof(next().c_str());
        } else if (arg == "--max-accel-mps2") {
            args.max_accel_mps2 = std::atof(next().c_str());
        } else if (arg == "--max-wheel-speed-rpm") {
            args.max_wheel_speed_rpm = std::atof(next().c_str());
        } else if (arg == "--hold-ms") {
            args.hold_ms = std::atof(next().c_str());
        } else if (arg == "--disable-ms") {
            args.disable_ms = std::atof(next().c_str());
        } else if (arg == "--stagger-ms") {
            args.stagger_ms = std::atof(next().c_str());
        } else if (arg == "--config") {
            args.config_path = next();
        } else if (arg == "--log-csv") {
            args.log_csv_path = next();
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

double counts_per_s_to_mps(const robot::RobotConfig &cfg, std::size_t module, std::int32_t counts_per_s) {
    double motor_rev_per_s = static_cast<double>(counts_per_s) / cfg.drive_encoder_counts_per_rev;
    double wheel_rev_per_s = motor_rev_per_s / cfg.drive_gear_ratio;
    double mps = wheel_rev_per_s * 2.0 * M_PI * cfg.wheel_radius_m();
    return cfg.drive_invert[module] ? -mps : mps;
}
std::int32_t mps_to_counts_per_s(const robot::RobotConfig &cfg, std::size_t module, double mps) {
    if (cfg.drive_invert[module]) mps = -mps;
    double wheel_rev_per_s = mps / (2.0 * M_PI * cfg.wheel_radius_m());
    double motor_rev_per_s = wheel_rev_per_s * cfg.drive_gear_ratio;
    return static_cast<std::int32_t>(std::lround(motor_rev_per_s * cfg.drive_encoder_counts_per_rev));
}

double clamp_magnitude(double value, double max_abs) { return std::clamp(value, -max_abs, max_abs); }

// Scales (vx, vy) down together, preserving direction, if their combined
// magnitude exceeds max_speed_mps -- clamping vx/vy independently instead
// would distort the commanded heading whenever only one axis needed
// clamping (e.g. vx=0.4, vy=0.15 clamped to 0.15 each becomes a 45-degree
// command instead of a slightly-slower version of the original heading).
kinematics::ChassisSpeeds clamp_translation(double vx_mps, double vy_mps, double max_speed_mps) {
    const double magnitude = std::hypot(vx_mps, vy_mps);
    if (magnitude <= max_speed_mps || magnitude <= 0.0) return {vx_mps, vy_mps, 0.0};
    const double scale = max_speed_mps / magnitude;
    return {vx_mps * scale, vy_mps * scale, 0.0};
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

    // CLI flag, if given, overrides the config file's value for this run.
    if (std::isnan(args.max_speed_mps)) args.max_speed_mps = cfg.max_speed_mps;
    if (std::isnan(args.max_omega_deg_s)) args.max_omega_deg_s = cfg.max_omega_deg_s;
    if (std::isnan(args.max_steer_rate_deg_s)) args.max_steer_rate_deg_s = cfg.max_steer_rate_deg_s;
    if (std::isnan(args.max_steer_accel_deg_s2)) args.max_steer_accel_deg_s2 = cfg.max_steer_accel_deg_s2;
    if (std::isnan(args.max_accel_mps2)) args.max_accel_mps2 = cfg.max_accel_mps2;
    if (std::isnan(args.max_wheel_speed_rpm)) args.max_wheel_speed_rpm = cfg.max_wheel_speed_rpm;
    for (std::size_t j = 0; j < args.modules.size(); ++j) {
        if (!args.have_module[j]) args.modules[j] = from_bus_location(cfg.module_bus_locations[j]);
    }

    const double max_omega_rad_s = args.max_omega_deg_s * M_PI / 180.0;
    // Wheel-speed cap uses the (possibly CLI-overridden) RPM value, not
    // cfg's directly, converted with the same wheel radius the drive
    // encoder scale uses.
    const double max_wheel_speed_mps =
        args.max_wheel_speed_rpm / 60.0 * 2.0 * M_PI * cfg.wheel_radius_m();

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
        zeroerr::configure_zeroerr_pdos(master, t.steer_slave, args.max_steer_rate_deg_s, args.max_steer_accel_deg_s2,
                                         args.max_steer_accel_deg_s2);
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
    // swerve_kinematics_test.cpp -- see those for why a single FaultReset
    // pulse isn't reliable enough on its own.
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

    constexpr std::size_t kTotalActuators = 8;
    const int stagger_cycles = std::max(0, static_cast<int>(args.stagger_ms / 1000.0 / kCycleSeconds));

    // Staggered enable, reusable both at startup and after a safety
    // disable triggered by a stale link (see the DISABLED watchdog state
    // below) -- same current-inrush fix as full_shake.cpp/
    // swerve_kinematics_test.cpp.
    auto stagger_enable_all = [&]() -> bool {
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
            if (all_ok) return true;
            next_wake += kCycle;
            std::this_thread::sleep_until(next_wake);
        }
        return false;
    };

    if (!stagger_enable_all()) {
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

    // Manual recovery, triggered by the web dashboard's "Clear Faults"
    // button. Deliberately more thorough than the live loop's own
    // periodic re-arm (below): that one only pulses FaultReset on
    // whichever actuators currently report a fault, which assumes the bus
    // itself is still at OPERATIONAL and that a plain fault_reset is
    // enough to resume -- neither is guaranteed. A fault_reset() alone
    // only clears the CiA-402 fault bit; per the CiA-402 state machine
    // that lands an axis in Switch On Disabled, not back in Operation
    // Enabled, and nothing in the live loop calls enable() again after
    // that. Separately, an E-stop/STO event can drop the whole EtherCAT
    // bus below OPERATIONAL, which fault_reset() has no ability to fix at
    // all -- only request_operational_state() can. This redoes the full
    // startup sequence (bus state check -> clear faults -> staggered
    // re-enable) on demand instead of requiring a process restart to get
    // both of those.
    auto manual_clear_faults = [&]() -> bool {
        std::printf("Clear-faults request received.\n");
        bool bus_operational = true;
        for (const auto &s : master.read_all_slave_states()) {
            if (s.al_state != 0x08 /* EC_STATE_OPERATIONAL */) {
                bus_operational = false;
                break;
            }
        }
        if (!bus_operational) {
            std::printf("  bus is not fully OPERATIONAL -- re-requesting (can take several seconds)...\n");
            if (!master.request_operational_state()) {
                std::fprintf(stderr, "  error: bus did not return to OPERATIONAL state\n");
                print_unhealthy_slaves(master);
                return false;
            }
            std::printf("  bus back at OPERATIONAL.\n");
        }
        update_all();
        master.send_receive();
        if (!try_clear_faults("manual clear-faults request")) {
            return false;
        }
        if (!stagger_enable_all()) {
            std::fprintf(stderr, "  error: not all actuators/axes reached OPERATION_ENABLED\n");
            return false;
        }
        std::printf("Clear-faults request succeeded -- all actuators operational.\n");
        return true;
    };

    kinematics::SwerveKinematics kinematics_solver(
        {module_positions[0], module_positions[1], module_positions[2], module_positions[3]});

    // last_commanded_angle_rad/last_commanded_speed_mps are the last
    // targets actually sent to hardware. Angle is kept as an unwrapped
    // accumulator (not normalized to [-pi,pi]) since the actuator's
    // absolute position target has no reason to wrap. Steering's
    // acceleration-limited ramp toward this angle now happens on the
    // actuator itself (Profile Position mode's Profile Accel/Decel, see
    // zeroerr_actuator.cpp) rather than in software here -- this used to
    // also be software-ramped, needed to avoid tripping 0x8400 "Velocity
    // Error Exceeds the Limit Value" on an instant velocity step, but
    // streaming raw CSP setpoints with no feedforward path turned out to
    // produce severely underdamped tracking on real hardware; Profile
    // Position mode's own on-drive profiler replaces both concerns at
    // once. The drive axis doesn't need either treatment: it's CSV
    // (velocity mode), so max_accel_mps2 below already limits the actual
    // commanded velocity directly, not a value merely implied by position
    // deltas.
    std::array<double, 4> last_commanded_angle_rad{};
    std::array<double, 4> last_commanded_speed_mps{0.0, 0.0, 0.0, 0.0};
    std::array<double, 4> commanded_raw_deg{};
    for (std::size_t j = 0; j < 4; ++j) {
        last_commanded_angle_rad[j] =
            cfg.raw_to_wheel_angle_deg(j, steer_actuators[j].snapshot().position_deg) * M_PI / 180.0;
    }

    // Per-cycle CSV of commanded vs. actual raw steer angle, for offline
    // comparison against joystick input -- see usage_and_exit()'s
    // --log-csv description. One row per 5ms cycle, not gated by
    // kPrintEveryNCycles, since the periodic printf is too coarse to
    // catch a brief mismatch (same reasoning as the edge-triggered
    // bit4/ack trace above).
    std::ofstream log_csv;
    if (!args.log_csv_path.empty()) {
        log_csv.open(args.log_csv_path);
        if (!log_csv) {
            std::fprintf(stderr, "error: could not open --log-csv path '%s'\n", args.log_csv_path.c_str());
            return 1;
        }
        log_csv << "t_ms,vx_cmd,vy_cmd,w_cmd";
        for (const char *m : kModuleNames) {
            // effort_raw/velocity_actual distinguish "not moving because
            // the drive isn't trying" (near-zero torque despite an
            // accepted move) from "not moving despite trying" (torque
            // saturated, position still frozen -- mechanically stuck).
            log_csv << ',' << m << "_target_deg," << m << "_actual_deg," << m << "_bit4," << m << "_ack," << m
                     << "_effort_raw," << m << "_velocity_actual";
        }
        log_csv << '\n';
    }

    bridge::ChassisLink chassis_link(args.command_port, args.telemetry_port, args.control_port);
    std::printf(
        "Listening for ChassisSpeeds commands on tcp://*:%d, publishing telemetry on tcp://*:%d, "
        "control (clear-faults) on tcp://*:%d\n",
        args.command_port, args.telemetry_port, args.control_port);
    std::printf(
        "Watchdog: hold last command for %.0fms, decay to zero by %.0fms, disable if silent past that.\n",
        args.hold_ms, args.disable_ms);

    kinematics::ChassisSpeeds last_received{};
    auto last_command_time = std::chrono::steady_clock::now();
    bool disabled_for_safety = false;
    constexpr int kTelemetryEveryNCycles = 10;   // ~50ms
    constexpr int kPrintEveryNCycles = 100;      // ~0.5s
    int cycle = 0;

    // Retriggerable version of the same stagger full_shake.cpp/
    // swerve_kinematics_test.cpp use at startup: those tools only ever
    // wake from rest ONCE (a scripted sequence with one beginning), so a
    // one-shot stagger was enough. A live joystick can go rest -> moving
    // -> rest -> moving repeatedly, and every one of those onsets is a
    // fresh synchronized-current-inrush risk if all 8 actuators start
    // ramping in the same PDO cycle -- confirmed on real hardware: the
    // very first nonzero drive command after this fix was missing faulted
    // all 4 modules simultaneously. wake_cycle is reset to "now" every
    // time the target goes from zero to nonzero; each module's steer/drive
    // gate only opens once its own stagger slot (steer 0..3, drive 4..7,
    // same grouping as elsewhere) has elapsed since that wake.
    bool prev_target_nonzero = false;
    int wake_cycle = 0;
    std::array<bool, 4> steer_was_faulted{false, false, false, false};
    std::array<bool, 4> drive_was_faulted{false, false, false, false};
    // Edge-triggered Profile Position handshake trace (see the print
    // below): the periodic per-cycle status print only samples once every
    // 100 cycles (500ms), far too coarse to see the Set-point Acknowledge
    // (statusword bit12), which the manual's own timing diagrams show as
    // a brief pulse -- easily missed entirely by that sampling rate even
    // when the handshake is working correctly. Tracking the previous
    // controlword bit4 ("New Set-point") and statusword bit12 per module
    // lets every actual transition get logged immediately, regardless of
    // the print cadence, without flooding the terminal (transitions are
    // inherently rate-limited by the handshake itself).
    std::array<bool, 4> prev_steer_bit4{false, false, false, false};
    std::array<bool, 4> prev_steer_ack{false, false, false, false};

    auto next_wake = std::chrono::steady_clock::now();
    while (!g_stop.load()) {
        // Checked regardless of disabled_for_safety/fault state below --
        // this is the whole point of a manual recovery action: it must
        // work even when the normal command path is currently blocked.
        // manual_clear_faults() blocks for a while (bus recovery alone can
        // take several seconds), so reset both timing references
        // afterward rather than let this cycle's stale next_wake/
        // last_command_time make the loop spin to catch up or the
        // watchdog immediately consider the very next cycle stale.
        if (chassis_link.try_receive_clear_faults_request()) {
            if (manual_clear_faults()) {
                disabled_for_safety = false;
                last_command_time = std::chrono::steady_clock::now();
            }
            next_wake = std::chrono::steady_clock::now();
        }

        auto received = chassis_link.try_receive_command();
        if (received.has_value()) {
            const bool was_disabled = disabled_for_safety;
            const kinematics::ChassisSpeeds clamped_translation =
                clamp_translation(received->vx_mps, received->vy_mps, args.max_speed_mps);
            last_received = kinematics::ChassisSpeeds{
                clamped_translation.vx_mps, clamped_translation.vy_mps,
                clamp_magnitude(received->omega_rad_per_s, max_omega_rad_s)};
            last_command_time = std::chrono::steady_clock::now();
            if (was_disabled) {
                std::printf("Fresh command received -- re-enabling...\n");
                if (!stagger_enable_all()) {
                    std::fprintf(stderr, "error: re-enable failed -- aborting\n");
                    return 1;
                }
                disabled_for_safety = false;
            }
        }

        const double age_ms =
            std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - last_command_time)
                .count();

        if (!disabled_for_safety && age_ms >= args.disable_ms) {
            std::printf("No command for %.0fms -- disabling for safety.\n", age_ms);
            for (auto &a : steer_actuators) a.disable();
            for (auto &a : drive_axes) a.disable();
            disabled_for_safety = true;
        }

        if (disabled_for_safety) {
            update_all();
            master.send_receive();
            next_wake += kCycle;
            std::this_thread::sleep_until(next_wake);
            ++cycle;
            continue;
        }

        // Belt-and-suspenders: keep nudging FaultReset if something
        // latches mid-session, without blocking this cycle's timing (a
        // real hardware fault must not stall the PDO exchange that keeps
        // the bus at OPERATIONAL).
        if (cycle % 40 == 0 && any_fault()) {
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

        const kinematics::ChassisSpeeds target =
            age_ms < args.hold_ms ? last_received : kinematics::ChassisSpeeds{};

        // Gated on the TARGET alone, not on whether any module still has
        // residual decaying speed: if the target is zero (stale-link decay
        // or a genuine commanded stop), every module should just coast its
        // speed to zero and hold its current heading -- not swing back to
        // face forward, which would be unrequested motion during what's
        // supposed to be a controlled stop.
        //
        // Threshold is deliberately well above float noise: a real
        // joystick's deadzone rescaling can still leak small values (e.g.
        // ~0.003-0.004 m/s) right at its edge even at rest -- confirmed in
        // the field, where that leakage was repeatedly re-arming (and so
        // effectively defeating) the stagger below well before the actual
        // commanded motion arrived. 1% of max_speed_mps/max_omega_rad_s
        // comfortably clears deadzone-edge leakage while still catching any
        // deliberate stick movement.
        const bool target_nonzero = std::hypot(target.vx_mps, target.vy_mps) > 0.01 * args.max_speed_mps ||
                                     std::abs(target.omega_rad_per_s) > 0.01 * max_omega_rad_s;
        if (target_nonzero && !prev_target_nonzero) {
            wake_cycle = cycle;
        }
        prev_target_nonzero = target_nonzero;

        std::array<kinematics::ModuleState, 4> desired =
            target_nonzero ? kinematics_solver.to_module_states(target) : std::array<kinematics::ModuleState, 4>{};
        // A combined translate+rotate target can ask one wheel to spin
        // faster than max_speed_mps/max_omega_deg_s alone imply -- derate
        // every module uniformly (not per-module) so heading stays correct.
        kinematics::SwerveKinematics::desaturate_wheel_speeds(desired, max_wheel_speed_mps);

        for (std::size_t j = 0; j < 4; ++j) {
            steer_actuators[j].update();
            drive_axes[j].update();

            {
                constexpr std::uint16_t kBitNewSetpoint = 1u << 4;
                constexpr std::uint16_t kBitSetpointAck = 1u << 12;
                auto steer_snap = steer_actuators[j].snapshot();
                const bool bit4 = (steer_snap.controlword_raw & kBitNewSetpoint) != 0;
                const bool ack = (steer_snap.statusword_raw & kBitSetpointAck) != 0;
                if (bit4 != prev_steer_bit4[j] || ack != prev_steer_ack[j]) {
                    std::printf("[%6.0fms] [%s steer] bit4: %d->%d  ack: %d->%d  (cw=0x%04X sw=0x%04X pos=%.2fdeg)\n",
                                cycle * kCycleSeconds * 1000.0, kModuleNames[j], prev_steer_bit4[j], bit4,
                                prev_steer_ack[j], ack, steer_snap.controlword_raw, steer_snap.statusword_raw,
                                steer_snap.position_deg);
                    prev_steer_bit4[j] = bit4;
                    prev_steer_ack[j] = ack;
                }
            }

            const bool steer_gate =
                target_nonzero && cycle >= wake_cycle + static_cast<int>(j) * stagger_cycles;
            const bool drive_gate =
                target_nonzero && cycle >= wake_cycle + static_cast<int>(4 + j) * stagger_cycles;

            kinematics::ModuleState optimized{};
            if (target_nonzero) {
                optimized = kinematics::SwerveKinematics::optimize(desired[j], last_commanded_angle_rad[j]);
            }

            // The steer actuator's own Profile Position move (Change Set
            // Immediately, see zeroerr_actuator.cpp) now does the
            // accel/velocity-limited ramp toward the target on-drive --
            // just hand it the kinematics-optimized angle directly. Not
            // steer_gate (at rest, or this module's stagger slot hasn't
            // opened yet) means holding the last commanded angle instead
            // of retargeting, so the module stays put rather than moving
            // before its staggered slot opens.
            if (steer_gate) {
                last_commanded_angle_rad[j] = optimized.angle_rad;
            }

            const double target_speed_mps = drive_gate ? optimized.speed_mps : 0.0;
            const double max_speed_step = args.max_accel_mps2 * kCycleSeconds;
            const double speed_delta = target_speed_mps - last_commanded_speed_mps[j];
            last_commanded_speed_mps[j] += clamp_magnitude(speed_delta, max_speed_step);

            commanded_raw_deg[j] = cfg.wheel_angle_to_raw_deg(j, last_commanded_angle_rad[j] * 180.0 / M_PI);
            steer_actuators[j].set_target_angle_deg(commanded_raw_deg[j]);
            drive_axes[j].set_target_velocity_counts_per_s(mps_to_counts_per_s(cfg, j, last_commanded_speed_mps[j]));

            // Rising-edge fault logging: the earlier bring-up checkpoints
            // print error codes, but this live loop's periodic re-arm
            // below did not -- meaning a real hardware fault here was
            // silently invisible except for a bare "FAULT" flag.
            const bool sf = steer_actuators[j].has_fault();
            if (sf && !steer_was_faulted[j]) {
                auto s = steer_actuators[j].snapshot();
                std::printf("[%s steer] Fault detected, error_code=0x%04X%s\n", kModuleNames[j], s.error_code,
                            s.sto_active ? ", STO_ACTIVE" : "");
            }
            steer_was_faulted[j] = sf;
            const bool df = drive_axes[j].has_fault();
            if (df && !drive_was_faulted[j]) {
                auto s = drive_axes[j].snapshot();
                std::printf("[%s drive] Fault detected, error_code=0x%04X%s\n", kModuleNames[j], s.error_code,
                            s.sto_active ? ", STO_ACTIVE" : "");
            }
            drive_was_faulted[j] = df;
        }
        master.send_receive();

        if (log_csv) {
            constexpr std::uint16_t kBitNewSetpoint = 1u << 4;
            constexpr std::uint16_t kBitSetpointAck = 1u << 12;
            log_csv << cycle * kCycleSeconds * 1000.0 << ',' << target.vx_mps << ',' << target.vy_mps << ','
                    << target.omega_rad_per_s;
            for (std::size_t j = 0; j < 4; ++j) {
                auto s = steer_actuators[j].snapshot();
                log_csv << ',' << commanded_raw_deg[j] << ',' << s.position_deg << ','
                        << ((s.controlword_raw & kBitNewSetpoint) != 0) << ','
                        << ((s.statusword_raw & kBitSetpointAck) != 0) << ',' << s.effort_actual_raw << ','
                        << s.velocity_actual_counts_per_s;
            }
            log_csv << '\n';
        }

        if (cycle % kTelemetryEveryNCycles == 0) {
            std::array<kinematics::ModuleState, 4> measured{};
            std::array<bridge::ModuleTelemetry, 4> module_telemetry{};
            for (std::size_t j = 0; j < 4; ++j) {
                auto steer_s = steer_actuators[j].snapshot();
                auto drive_s = drive_axes[j].snapshot();
                const double wheel_angle_deg = cfg.raw_to_wheel_angle_deg(j, steer_s.position_deg);
                const double wheel_speed_mps = counts_per_s_to_mps(cfg, j, drive_s.velocity_actual_counts_per_s);
                measured[j] = kinematics::ModuleState{wheel_speed_mps, wheel_angle_deg * M_PI / 180.0};
                module_telemetry[j] = bridge::ModuleTelemetry{wheel_angle_deg, wheel_speed_mps,
                                                                steer_s.has_fault || drive_s.has_fault};
            }
            kinematics::ChassisSpeeds odom = kinematics_solver.to_chassis_speeds(measured);
            chassis_link.publish_telemetry(odom, any_fault(), module_telemetry);
        }

        if (cycle % kPrintEveryNCycles == 0) {
            std::printf("[age=%5.0fms] cmd(vx=%6.3f vy=%6.3f w=%6.2f)\n", age_ms, last_received.vx_mps,
                        last_received.vy_mps, last_received.omega_rad_per_s);
            for (std::size_t j = 0; j < 4; ++j) {
                auto steer_s = steer_actuators[j].snapshot();
                auto drive_s = drive_axes[j].snapshot();
                // bit4 (New Set-point) and bit12 (Set-point Acknowledge)
                // decoded explicitly -- diagnosing whether Profile
                // Position moves are actually being accepted, or the
                // retarget handshake (zeroerr_actuator.cpp) is stuck
                // waiting on an acknowledge that isn't arriving as
                // expected. Not needed once that's confirmed working.
                constexpr std::uint16_t kBitNewSetpoint = 1u << 4;
                constexpr std::uint16_t kBitSetpointAck = 1u << 12;
                std::printf(
                    "    [%s] steer=%7.2fdeg mode=%d cw=0x%04X(bit4=%d) sw=0x%04X(ack=%d) drive_cmd=%8d "
                    "drive_act=%8d fault=%s\n",
                    kModuleNames[j], steer_s.position_deg, steer_s.mode_of_operation_display,
                    steer_s.controlword_raw, (steer_s.controlword_raw & kBitNewSetpoint) != 0,
                    steer_s.statusword_raw, (steer_s.statusword_raw & kBitSetpointAck) != 0,
                    drive_s.commanded_velocity_counts_per_s, drive_s.velocity_actual_counts_per_s,
                    (steer_s.has_fault || drive_s.has_fault) ? "FAULT" : "-");
            }
            std::fflush(stdout);
        }

        ++cycle;
        next_wake += kCycle;
        std::this_thread::sleep_until(next_wake);
    }

    std::printf("Ramping drives down to zero before disabling...\n");
    const int ramp_down_cycles = cycles_for(1.0);
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
