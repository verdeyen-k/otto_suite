// Combined 8-actuator soak test: continuously steers ZeroErr eRobs through
// a +/-90deg triangle-wave oscillation while simultaneously running Copley
// drive axes through a +/-500000 counts/s triangle-wave oscillation, all in
// one shared EtherCAT session. Exercises plan.md's "run multiple
// axes/actuators together" milestone across BOTH drive types at once,
// under continuous motion rather than a single ramp-then-hold move.
//
// Run with a human physically present, hand on the E-stop, wheels off the
// ground / free to spin, steering free to rotate.
//
// Usage:
//   sudo ./full_shake --iface enp1s0 \
//     --steer slave=5 --steer slave=6 --steer slave=8 --steer slave=9 \
//     --drive slave=2,axis=a --drive slave=2,axis=b \
//     --drive slave=3,axis=a --drive slave=3,axis=b \
//     [--steer-amplitude-deg 90] [--steer-period-s 4] \
//     [--drive-amplitude 500000] [--drive-period-s 10] \
//     [--duration-s 20] [--ramp-s 1] [--stagger-ms 100]
//
// --duration-s is the length of the continuous oscillation phase, AFTER
// the initial --ramp-s approach to the wave's starting point (so total
// run time is ramp_s + duration_s). Both waves are symmetric triangle
// waves around zero, starting at +amplitude (steering starts fully one
// way, drive starts ramping toward full speed one way) and swinging to
// -amplitude and back, repeating.
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "cia402/state_machine.hpp"
#include "copley/copley_axis.hpp"
#include "copley/copley_identity.hpp"
#include "ethercat/soem_master.hpp"
#include "zeroerr/zeroerr_actuator.hpp"
#include "zeroerr/zeroerr_identity.hpp"

namespace {

std::atomic<bool> g_stop{false};
void on_sigint(int) { g_stop.store(true); }

struct SteerTarget {
    int slave_index = -1;
};

struct DriveTarget {
    int slave_index = -1;
    copley::Axis axis = copley::Axis::A;
};

struct Args {
    std::string iface;
    std::vector<SteerTarget> steers;
    std::vector<DriveTarget> drives;
    double steer_amplitude_deg = 90.0;
    double steer_period_s = 4.0;
    double drive_amplitude = 500000.0;
    double drive_period_s = 10.0;
    double duration_s = 20.0;
    double ramp_s = 1.0;
    double stagger_ms = 100.0;
};

[[noreturn]] void usage_and_exit(const char *prog) {
    std::fprintf(stderr,
                 "Usage: %s --iface IFNAME --steer slave=N [--steer ...] "
                 "--drive slave=N,axis={a|b} [--drive ...]\n"
                 "         [--steer-amplitude-deg 90] [--steer-period-s 4]\n"
                 "         [--drive-amplitude 500000] [--drive-period-s 10]\n"
                 "         [--duration-s 20] [--ramp-s 1] [--stagger-ms 100]\n"
                 "  At least one --steer or --drive is required.\n"
                 "  Both waves are symmetric triangle waves around zero, starting at\n"
                 "  +amplitude and swinging to -amplitude and back, repeating.\n"
                 "  duration-s is the oscillation phase length, AFTER the initial\n"
                 "  ramp-s approach to the wave's starting point.\n",
                 prog);
    std::exit(2);
}

const char *axis_name(copley::Axis axis) { return axis == copley::Axis::A ? "A" : "B"; }

SteerTarget parse_steer(const std::string &spec, const char *prog) {
    SteerTarget t;
    bool have_slave = false;
    std::stringstream ss(spec);
    std::string kv;
    while (std::getline(ss, kv, ',')) {
        auto eq = kv.find('=');
        if (eq == std::string::npos) usage_and_exit(prog);
        std::string key = kv.substr(0, eq);
        std::string value = kv.substr(eq + 1);
        if (key == "slave") {
            t.slave_index = std::atoi(value.c_str());
            have_slave = true;
        } else {
            std::fprintf(stderr, "error: unknown --steer key '%s'\n", key.c_str());
            usage_and_exit(prog);
        }
    }
    if (!have_slave) usage_and_exit(prog);
    return t;
}

DriveTarget parse_drive(const std::string &spec, const char *prog) {
    DriveTarget t;
    bool have_slave = false;
    bool have_axis = false;
    std::stringstream ss(spec);
    std::string kv;
    while (std::getline(ss, kv, ',')) {
        auto eq = kv.find('=');
        if (eq == std::string::npos) usage_and_exit(prog);
        std::string key = kv.substr(0, eq);
        std::string value = kv.substr(eq + 1);
        if (key == "slave") {
            t.slave_index = std::atoi(value.c_str());
            have_slave = true;
        } else if (key == "axis") {
            if (value == "a" || value == "A") {
                t.axis = copley::Axis::A;
            } else if (value == "b" || value == "B") {
                t.axis = copley::Axis::B;
            } else {
                usage_and_exit(prog);
            }
            have_axis = true;
        } else {
            std::fprintf(stderr, "error: unknown --drive key '%s'\n", key.c_str());
            usage_and_exit(prog);
        }
    }
    if (!have_slave || !have_axis) usage_and_exit(prog);
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
        } else if (arg == "--steer") {
            args.steers.push_back(parse_steer(next(), argv[0]));
        } else if (arg == "--drive") {
            args.drives.push_back(parse_drive(next(), argv[0]));
        } else if (arg == "--steer-amplitude-deg") {
            args.steer_amplitude_deg = std::atof(next().c_str());
        } else if (arg == "--steer-period-s") {
            args.steer_period_s = std::atof(next().c_str());
        } else if (arg == "--drive-amplitude") {
            args.drive_amplitude = std::atof(next().c_str());
        } else if (arg == "--drive-period-s") {
            args.drive_period_s = std::atof(next().c_str());
        } else if (arg == "--duration-s") {
            args.duration_s = std::atof(next().c_str());
        } else if (arg == "--ramp-s") {
            args.ramp_s = std::atof(next().c_str());
        } else if (arg == "--stagger-ms") {
            args.stagger_ms = std::atof(next().c_str());
        } else {
            usage_and_exit(argv[0]);
        }
    }
    if (args.iface.empty() || (args.steers.empty() && args.drives.empty())) usage_and_exit(argv[0]);
    return args;
}

constexpr auto kCycle = std::chrono::microseconds(5000);
constexpr double kCycleSeconds = kCycle.count() / 1e6;

int cycles_for(double seconds) { return static_cast<int>(seconds / kCycleSeconds); }

// Symmetric triangle wave: starts at +amplitude at t=0, reaches
// -amplitude at t=period/2, back to +amplitude at t=period, repeating.
double triangle_wave(double t, double period, double amplitude) {
    if (period <= 0.0) return amplitude;
    double phase = std::fmod(t, period) / period;
    if (phase < 0.0) phase += 1.0;
    if (phase < 0.5) {
        return amplitude * (1.0 - 4.0 * phase);
    }
    return amplitude * (4.0 * phase - 3.0);
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

    ethercat::SoemMaster master(args.iface);
    int slave_count;
    try {
        slave_count = master.scan();
    } catch (const std::exception &e) {
        std::fprintf(stderr, "error: %s\n", e.what());
        return 1;
    }
    std::printf("Found %d slave(s) on '%s'.\n", slave_count, args.iface.c_str());

    for (const auto &t : args.steers) {
        std::printf("Steering slave [%d] name='%s' +/-%.1fdeg, %.1fs period.\n", t.slave_index,
                     master.slave_name(t.slave_index).c_str(), args.steer_amplitude_deg, args.steer_period_s);
        zeroerr::configure_zeroerr_pdos(master, t.slave_index);
    }
    // configure_copley_pdos() configures BOTH axes of a slave at once --
    // register it once per unique slave, not once per target.
    std::vector<int> configured_drive_slaves;
    for (const auto &t : args.drives) {
        std::printf("Driving slave [%d] name='%s' axis=%s +/-%.0f counts/s, %.1fs period.\n", t.slave_index,
                     master.slave_name(t.slave_index).c_str(), axis_name(t.axis), args.drive_amplitude,
                     args.drive_period_s);
        if (std::find(configured_drive_slaves.begin(), configured_drive_slaves.end(), t.slave_index) ==
            configured_drive_slaves.end()) {
            copley::configure_copley_pdos(master, t.slave_index);
            configured_drive_slaves.push_back(t.slave_index);
        }
    }
    master.configure_pdos();

    if (!master.wait_for_safe_op()) {
        std::fprintf(stderr, "error: bus did not reach SAFE_OP -- aborting\n");
        print_unhealthy_slaves(master);
        return 1;
    }

    std::vector<zeroerr::ZeroErrActuator> steer_actuators;
    steer_actuators.reserve(args.steers.size());
    for (const auto &t : args.steers) {
        steer_actuators.emplace_back(master, t.slave_index);
    }
    std::vector<copley::CopleyAxis> drive_axes;
    drive_axes.reserve(args.drives.size());
    for (const auto &t : args.drives) {
        drive_axes.emplace_back(master, t.slave_index, t.axis);
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
    auto print_fault = [&](int slave, const char *label, std::uint16_t error_code, bool sto_active,
                            const char *context) {
        std::printf("%s Fault detected (%s, error_code=0x%04X%s). Resetting...\n", label, context, error_code,
                    sto_active ? ", STO_ACTIVE" : "");
    };
    // Checks every steer actuator and drive axis for a fault and, if any
    // is present, re-arms the FaultReset pulse periodically (a single
    // pulse can fail to land or the fault can re-trigger right as it's
    // processed -- see zeroerr_move_all.cpp/copley_move_all.cpp for the
    // same fix) for up to 1s. Returns true if nothing was faulted or it
    // all cleared.
    auto try_clear_faults = [&](const char *context) -> bool {
        if (!any_fault()) {
            return true;
        }
        for (std::size_t i = 0; i < steer_actuators.size(); ++i) {
            if (steer_actuators[i].has_fault()) {
                auto s = steer_actuators[i].snapshot();
                char label[32];
                std::snprintf(label, sizeof(label), "[steer %d]", args.steers[i].slave_index);
                print_fault(args.steers[i].slave_index, label, s.error_code, s.sto_active, context);
                steer_actuators[i].fault_reset();
            }
        }
        for (std::size_t i = 0; i < drive_axes.size(); ++i) {
            if (drive_axes[i].has_fault()) {
                auto s = drive_axes[i].snapshot();
                char label[32];
                std::snprintf(label, sizeof(label), "[drive %d/%s]", args.drives[i].slave_index,
                              axis_name(args.drives[i].axis));
                print_fault(args.drives[i].slave_index, label, s.error_code, s.sto_active, context);
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
                std::fprintf(stderr, "error: steer slave [%d] fault did not clear (%s) -- aborting\n",
                             args.steers[i].slave_index, context);
                all_clear = false;
            }
        }
        for (std::size_t i = 0; i < drive_axes.size(); ++i) {
            if (drive_axes[i].has_fault()) {
                std::fprintf(stderr, "error: drive slave [%d/%s] fault did not clear (%s) -- aborting\n",
                             args.drives[i].slave_index, axis_name(args.drives[i].axis), context);
                all_clear = false;
            }
        }
        return all_clear;
    };

    update_all();
    master.send_receive();

    // Deliberately NOT aborting if this fails: AL state and the CiA-402
    // application fault are separate layers -- confirmed on real hardware
    // that a ZeroErr eRob's 0xA000 ("master offline") fault does not
    // clear while still at SAFE_OP, plausibly because its firmware only
    // considers the master genuinely online once real OPERATIONAL cyclic
    // exchange is happening. See zeroerr_move.cpp/zeroerr_move_all.cpp.
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

    // Combined stagger order across all actuators, steer first then
    // drive: enabling several actuators in the same cycle means each
    // one's startup current inrush hits the shared power rail at once --
    // confirmed on real hardware, 0x3220 "bus voltage undervoltage"
    // tripping on multiple ZeroErr eRobs enabled together. See
    // zeroerr_move_all.cpp/copley_move_all.cpp for the same fix.
    const std::size_t total_actuators = steer_actuators.size() + drive_axes.size();
    const int stagger_cycles = std::max(0, static_cast<int>(args.stagger_ms / 1000.0 / kCycleSeconds));
    auto enable_nth = [&](std::size_t n) {
        if (n < steer_actuators.size()) {
            std::printf("  [steer %d] enabling\n", args.steers[n].slave_index);
            steer_actuators[n].enable();
        } else {
            std::size_t d = n - steer_actuators.size();
            std::printf("  [drive %d/%s] enabling\n", args.drives[d].slave_index, axis_name(args.drives[d].axis));
            drive_axes[d].enable();
        }
    };
    auto nth_operational = [&](std::size_t n) {
        if (n < steer_actuators.size()) return steer_actuators[n].is_operational();
        return drive_axes[n - steer_actuators.size()].is_operational();
    };

    std::printf("Enabling %zu actuator(s)/axis(es), staggered %.0fms apart...\n", total_actuators, args.stagger_ms);
    std::size_t next_to_enable = 0;
    bool all_operational = false;
    auto next_wake = std::chrono::steady_clock::now();
    for (int i = 0; i < cycles_for(5.0) && !g_stop.load(); ++i) {
        if (next_to_enable < total_actuators && i >= static_cast<int>(next_to_enable) * stagger_cycles) {
            enable_nth(next_to_enable);
            ++next_to_enable;
        }
        update_all();
        master.send_receive();
        bool all_ok = true;
        for (std::size_t j = 0; j < total_actuators; ++j) {
            if (j >= next_to_enable || !nth_operational(j)) {
                all_ok = false;
            }
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
                std::fprintf(stderr, "  steer [%d] state=%s\n", args.steers[j].slave_index,
                             std::string(cia402::to_string(steer_actuators[j].snapshot().state)).c_str());
            }
        }
        for (std::size_t j = 0; j < drive_axes.size(); ++j) {
            if (!drive_axes[j].is_operational()) {
                std::fprintf(stderr, "  drive [%d/%s] state=%s\n", args.drives[j].slave_index,
                             axis_name(args.drives[j].axis),
                             std::string(cia402::to_string(drive_axes[j].snapshot().state)).c_str());
            }
        }
        return 1;
    }

    std::vector<double> steer_start_degs(steer_actuators.size());
    for (std::size_t j = 0; j < steer_actuators.size(); ++j) {
        steer_start_degs[j] = steer_actuators[j].snapshot().position_deg;
    }

    std::printf("Ramping to start over %.2fs, then oscillating for %.2fs...\n", args.ramp_s, args.duration_s);

    // Each actuator/axis starts being actively commanded (and its own
    // ramp+oscillation clock starts) at its own staggered offset, not all
    // at cycle 0 -- see zeroerr_move_all.cpp for why the ramp/wave start
    // itself, not just enable(), needs the same staggering (a ZeroErr
    // actuator's target silently tracks its actual position until first
    // commanded, so that first command is a real synchronized mode-switch
    // if done for every actuator at once).
    const int total_cycles =
        cycles_for(args.ramp_s + args.duration_s) + static_cast<int>(total_actuators - 1) * stagger_cycles;
    next_wake = std::chrono::steady_clock::now();
    for (int i = 0; i < total_cycles && !g_stop.load(); ++i) {
        for (std::size_t j = 0; j < steer_actuators.size(); ++j) {
            steer_actuators[j].update();
            int start_cycle = static_cast<int>(j) * stagger_cycles;
            if (i >= start_cycle) {
                double elapsed_s = (i - start_cycle) * kCycleSeconds;
                double target_deg;
                if (elapsed_s < args.ramp_s) {
                    double fraction = args.ramp_s > 0.0 ? elapsed_s / args.ramp_s : 1.0;
                    target_deg =
                        steer_start_degs[j] + (args.steer_amplitude_deg - steer_start_degs[j]) * fraction;
                } else {
                    target_deg = triangle_wave(elapsed_s - args.ramp_s, args.steer_period_s, args.steer_amplitude_deg);
                }
                steer_actuators[j].set_target_angle_deg(target_deg);
            }
        }
        for (std::size_t j = 0; j < drive_axes.size(); ++j) {
            drive_axes[j].update();
            int start_cycle = static_cast<int>(steer_actuators.size() + j) * stagger_cycles;
            if (i >= start_cycle) {
                double elapsed_s = (i - start_cycle) * kCycleSeconds;
                double target_vel;
                if (elapsed_s < args.ramp_s) {
                    double fraction = args.ramp_s > 0.0 ? elapsed_s / args.ramp_s : 1.0;
                    target_vel = args.drive_amplitude * fraction;
                } else {
                    target_vel = triangle_wave(elapsed_s - args.ramp_s, args.drive_period_s, args.drive_amplitude);
                }
                drive_axes[j].set_target_velocity_counts_per_s(static_cast<std::int32_t>(target_vel));
            }
        }
        master.send_receive();

        // Overwrite the same block in place instead of scrolling -- same
        // style as zeroerr_state/copley_state/the other _all tools.
        const std::size_t lines = steer_actuators.size() + drive_axes.size();
        if (i != 0) {
            std::printf("\033[%zuA", lines);
        }
        for (std::size_t j = 0; j < steer_actuators.size(); ++j) {
            auto s = steer_actuators[j].snapshot();
            std::printf("  [steer %d] pos=%7.2f vel=%7.2f deg/s fault=%s\033[K\n", args.steers[j].slave_index,
                        s.position_deg, s.velocity_deg_per_s,
                        s.has_fault ? (s.sto_active ? "STO_ACTIVE" : "FAULT") : "-");
        }
        for (std::size_t j = 0; j < drive_axes.size(); ++j) {
            auto s = drive_axes[j].snapshot();
            std::printf("  [drive %d/%s] cmd=%8d vel=%8d fault=%s\033[K\n", args.drives[j].slave_index,
                        axis_name(args.drives[j].axis), s.commanded_velocity_counts_per_s,
                        s.velocity_actual_counts_per_s, s.has_fault ? (s.sto_active ? "STO_ACTIVE" : "FAULT") : "-");
        }
        std::fflush(stdout);
        next_wake += kCycle;
        std::this_thread::sleep_until(next_wake);
    }

    std::printf("Ramping drives down to zero before disabling...\n");
    const int ramp_down_cycles = cycles_for(args.ramp_s > 0.0 ? args.ramp_s : 1.0);
    next_wake = std::chrono::steady_clock::now();
    for (int i = 0; i < ramp_down_cycles; ++i) {
        double fraction = 1.0 - static_cast<double>(i) / ramp_down_cycles;
        for (auto &a : steer_actuators) {
            a.update();
        }
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
