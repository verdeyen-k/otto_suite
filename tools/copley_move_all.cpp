// Multi-axis bring-up tool: brings ALL specified Copley BE2 axes through
// the DS402 enable sequence together, in one EtherCAT session, and commands
// each to its own target velocity, ramping synchronously in time. Extension
// of copley_move to multiple axes at once (plan.md's "test multiple
// axes/actuators together" milestone, once each individual axis is already
// proven). Run with a human physically present, hand on the E-stop, wheels
// free to spin (off the ground / unloaded).
//
// Usage:
//   sudo ./copley_move_all --iface enp1s0 \
//     --target slave=2,axis=a,velocity=50000 --target slave=2,axis=b,velocity=-50000 \
//     [--duration-s 5] [--ramp-s 1]
//   --target is repeatable; at least one is required. Velocity is raw
//   encoder counts/s (see copley_move.cpp for why no RPM/m/s conversion is
//   claimed).
#include <algorithm>
#include <atomic>
#include <chrono>
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

namespace {

std::atomic<bool> g_stop{false};
void on_sigint(int) { g_stop.store(true); }

struct TargetSpec {
    int slave_index = -1;
    copley::Axis axis = copley::Axis::A;
    std::int32_t velocity_counts_per_s = 0;
};

struct Args {
    std::string iface;
    std::vector<TargetSpec> targets;
    double duration_s = 5.0;
    double ramp_s = -1.0;  // -1 means min(1.0, duration_s / 2)
};

[[noreturn]] void usage_and_exit(const char *prog) {
    std::fprintf(stderr,
                 "Usage: %s --iface IFNAME --target slave=N,axis={a|b},velocity=COUNTS_PER_S "
                 "[--target ...] [--duration-s S] [--ramp-s S]\n"
                 "  --target is repeatable; at least one is required.\n"
                 "  Velocity is raw encoder counts/s -- no RPM/m/s conversion is claimed.\n",
                 prog);
    std::exit(2);
}

const char *axis_name(copley::Axis axis) { return axis == copley::Axis::A ? "A" : "B"; }

TargetSpec parse_target(const std::string &spec, const char *prog) {
    TargetSpec t;
    bool have_slave = false;
    bool have_axis = false;
    bool have_velocity = false;
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
        } else if (key == "velocity") {
            t.velocity_counts_per_s = std::atoi(value.c_str());
            have_velocity = true;
        } else {
            std::fprintf(stderr, "error: unknown --target key '%s'\n", key.c_str());
            usage_and_exit(prog);
        }
    }
    if (!have_slave || !have_axis || !have_velocity) usage_and_exit(prog);
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
        } else if (arg == "--target") {
            args.targets.push_back(parse_target(next(), argv[0]));
        } else if (arg == "--duration-s") {
            args.duration_s = std::atof(next().c_str());
        } else if (arg == "--ramp-s") {
            args.ramp_s = std::atof(next().c_str());
        } else {
            usage_and_exit(argv[0]);
        }
    }
    if (args.iface.empty() || args.targets.empty()) usage_and_exit(argv[0]);
    if (args.ramp_s < 0.0) args.ramp_s = std::min(1.0, args.duration_s / 2.0);
    return args;
}

constexpr auto kCycle = std::chrono::microseconds(5000);
constexpr double kCycleSeconds = kCycle.count() / 1e6;

int cycles_for(double seconds) { return static_cast<int>(seconds / kCycleSeconds); }

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

// Mirrors the identical helper in copley_move.cpp, generalized to
// multiple axes: checks every axis for a fault and, if any is present,
// pulses fault-reset + clears the Copley-specific latching fault register
// on just those, then waits up to 1s for it to clear.
bool try_clear_faults(std::vector<copley::CopleyAxis> &axes, const std::vector<TargetSpec> &targets,
                       ethercat::SoemMaster &master, const char *context) {
    bool any_fault = false;
    for (std::size_t i = 0; i < axes.size(); ++i) {
        if (axes[i].has_fault()) {
            auto s = axes[i].snapshot();
            std::printf("[%d/%s] Fault detected (%s, error_code=0x%04X%s). Resetting...\n", targets[i].slave_index,
                        axis_name(targets[i].axis), context, s.error_code, s.sto_active ? ", STO_ACTIVE" : "");
            axes[i].fault_reset();
            axes[i].clear_latching_faults();
            any_fault = true;
        }
    }
    if (!any_fault) {
        return true;
    }
    // fault_reset() only arms a SINGLE rising-edge pulse of the FaultReset
    // controlword bit (see cia402::StateMachine::next_controlword_bits())
    // -- if that one pulse doesn't land (a dropped frame, or the fault
    // re-triggering right as it's processed), nothing resends it for the
    // rest of this wait. Re-arm it (and re-clear the latching fault
    // register) periodically for whichever axes are still faulted, the
    // same way request_operational_state() resends its own one-shot
    // request rather than sending it only once.
    constexpr int kRearmEveryNCycles = 40;  // ~200ms at the 5ms cycle
    auto next_wake = std::chrono::steady_clock::now();
    for (int i = 0; i < cycles_for(1.0) && !g_stop.load(); ++i) {
        if (i % kRearmEveryNCycles == 0) {
            for (auto &a : axes) {
                if (a.has_fault()) {
                    a.fault_reset();
                    a.clear_latching_faults();
                }
            }
        }
        for (auto &a : axes) {
            a.update();
        }
        master.send_receive();
        next_wake += kCycle;
        std::this_thread::sleep_until(next_wake);
    }
    bool all_clear = true;
    for (std::size_t i = 0; i < axes.size(); ++i) {
        if (axes[i].has_fault()) {
            std::fprintf(stderr, "error: slave [%d/%s] fault did not clear (%s) -- aborting\n",
                         targets[i].slave_index, axis_name(targets[i].axis), context);
            all_clear = false;
        }
    }
    return all_clear;
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

    // configure_copley_pdos() configures BOTH axes of a slave at once --
    // register it once per unique slave, not once per target (a slave
    // normally has two targets, one per axis).
    std::vector<int> configured_slaves;
    for (const auto &t : args.targets) {
        std::printf("Targeting slave [%d] name='%s' axis=%s velocity=%d counts/s.\n", t.slave_index,
                     master.slave_name(t.slave_index).c_str(), axis_name(t.axis), t.velocity_counts_per_s);
        if (std::find(configured_slaves.begin(), configured_slaves.end(), t.slave_index) ==
            configured_slaves.end()) {
            copley::configure_copley_pdos(master, t.slave_index);
            configured_slaves.push_back(t.slave_index);
        }
    }
    master.configure_pdos();

    if (!master.wait_for_safe_op()) {
        std::fprintf(stderr, "error: bus did not reach SAFE_OP -- aborting\n");
        print_unhealthy_slaves(master);
        return 1;
    }

    std::vector<copley::CopleyAxis> axes;
    axes.reserve(args.targets.size());
    for (const auto &t : args.targets) {
        axes.emplace_back(master, t.slave_index, t.axis);
    }

    for (auto &a : axes) {
        a.update();
    }
    master.send_receive();

    if (!try_clear_faults(axes, args.targets, master, "at SAFE_OP")) {
        return 1;
    }

    if (!master.request_operational_state()) {
        std::fprintf(stderr, "error: bus did not reach OPERATIONAL state -- aborting\n");
        print_unhealthy_slaves(master);
        return 1;
    }

    for (auto &a : axes) {
        a.update();
    }
    master.send_receive();
    if (!try_clear_faults(axes, args.targets, master, "at OPERATIONAL, before any enable command")) {
        return 1;
    }

    for (auto &a : axes) {
        a.enable();
    }
    std::printf("Enabling %zu axis/axes...\n", axes.size());
    std::vector<cia402::DriveState> last_states(axes.size());
    for (std::size_t i = 0; i < axes.size(); ++i) {
        last_states[i] = axes[i].snapshot().state;
    }
    bool all_operational = false;
    auto next_wake = std::chrono::steady_clock::now();
    for (int i = 0; i < cycles_for(5.0) && !g_stop.load(); ++i) {
        for (auto &a : axes) {
            a.update();
        }
        master.send_receive();
        bool all_ok = true;
        for (std::size_t j = 0; j < axes.size(); ++j) {
            auto state = axes[j].snapshot().state;
            if (state != last_states[j]) {
                std::printf("  [%d/%s] state -> %s\n", args.targets[j].slave_index, axis_name(args.targets[j].axis),
                            std::string(cia402::to_string(state)).c_str());
                last_states[j] = state;
            }
            if (!axes[j].is_operational()) {
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
        std::fprintf(stderr, "error: not all axes reached OPERATION_ENABLED within 5s -- aborting\n");
        for (std::size_t j = 0; j < axes.size(); ++j) {
            if (!axes[j].is_operational()) {
                std::fprintf(stderr, "  slave [%d/%s] state=%s\n", args.targets[j].slave_index,
                             axis_name(args.targets[j].axis),
                             std::string(cia402::to_string(axes[j].snapshot().state)).c_str());
            }
        }
        return 1;
    }

    std::printf("Commanding %zu axis/axes (ramping over %.2fs, holding %.2fs total)...\n", axes.size(), args.ramp_s,
                args.duration_s);

    const int total_cycles = cycles_for(args.duration_s);
    next_wake = std::chrono::steady_clock::now();
    for (int i = 0; i < total_cycles && !g_stop.load(); ++i) {
        double elapsed_s = i * kCycleSeconds;
        double fraction = args.ramp_s > 0.0 ? std::min(1.0, elapsed_s / args.ramp_s) : 1.0;

        for (std::size_t j = 0; j < axes.size(); ++j) {
            auto command = static_cast<std::int32_t>(args.targets[j].velocity_counts_per_s * fraction);
            axes[j].update();
            axes[j].set_target_velocity_counts_per_s(command);
        }
        master.send_receive();

        if (i != 0) {
            std::printf("\033[%zuA", axes.size());
        }
        for (std::size_t j = 0; j < axes.size(); ++j) {
            auto s = axes[j].snapshot();
            std::printf("  [%d/%s] cmd=%8d vel=%8d pos=%10d foll_err=%7d torque=%6d fault=%s\033[K\n",
                        args.targets[j].slave_index, axis_name(args.targets[j].axis), s.commanded_velocity_counts_per_s,
                        s.velocity_actual_counts_per_s, s.position_actual_counts, s.following_error_counts,
                        s.torque_actual_raw, s.has_fault ? (s.sto_active ? "STO_ACTIVE" : "FAULT") : "-");
        }
        std::fflush(stdout);
        next_wake += kCycle;
        std::this_thread::sleep_until(next_wake);
    }

    std::printf("Ramping down to zero before disabling...\n");
    const int ramp_down_cycles = cycles_for(args.ramp_s > 0.0 ? args.ramp_s : 1.0);
    next_wake = std::chrono::steady_clock::now();
    for (int i = 0; i < ramp_down_cycles; ++i) {
        double fraction = 1.0 - static_cast<double>(i) / ramp_down_cycles;
        for (std::size_t j = 0; j < axes.size(); ++j) {
            auto command = static_cast<std::int32_t>(args.targets[j].velocity_counts_per_s * fraction);
            axes[j].update();
            axes[j].set_target_velocity_counts_per_s(command);
        }
        master.send_receive();
        next_wake += kCycle;
        std::this_thread::sleep_until(next_wake);
    }

    std::printf("Disabling and closing bus.\n");
    for (auto &a : axes) {
        a.disable();
    }
    next_wake = std::chrono::steady_clock::now();
    for (int i = 0; i < cycles_for(1.0); ++i) {
        for (auto &a : axes) {
            a.update();
        }
        master.send_receive();
        next_wake += kCycle;
        std::this_thread::sleep_until(next_wake);
    }
    master.close();
    return 0;
}
