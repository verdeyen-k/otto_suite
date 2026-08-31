// Multi-actuator bring-up tool: brings ALL specified ZeroErr eRobs through
// the DS402 enable sequence together, in one EtherCAT session, and commands
// each to its own absolute target angle, ramping synchronously in time.
// Extension of zeroerr_move to multiple actuators at once (plan.md's "test
// multiple actuators together" milestone, once each individual actuator is
// already proven). Run with a human physically present, hand on the
// E-stop, mechanisms free to rotate.
//
// Usage:
//   sudo ./zeroerr_move_all --iface enp1s0 \
//     --target slave=5,angle=30 --target slave=6,angle=-15 \
//     [--duration-s 5] [--ramp-s 1]
//   --target is repeatable; at least one is required. angle is the
//   ABSOLUTE target angle for that actuator (not a delta).
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "cia402/state_machine.hpp"
#include "ethercat/soem_master.hpp"
#include "zeroerr/steer_position_controller.hpp"
#include "zeroerr/zeroerr_actuator.hpp"
#include "zeroerr/zeroerr_identity.hpp"

namespace {

std::atomic<bool> g_stop{false};
void on_sigint(int) { g_stop.store(true); }

struct TargetSpec {
    int slave_index = -1;
    double angle_deg = 0.0;
};

struct Args {
    std::string iface;
    std::vector<TargetSpec> targets;
    double duration_s = 5.0;
    double ramp_s = -1.0;  // -1 means min(1.0, duration_s / 2)
    // Enabling several actuators in the same cycle means each one's
    // startup current inrush hits the shared power rail at once --
    // confirmed on real hardware: enabling 4 eRobs together tripped
    // 0x3220 ("bus voltage undervoltage", manual p.120) on all 4
    // simultaneously. Staggering each actuator's enable() call spreads
    // that inrush out over time instead.
    double stagger_ms = 100.0;
};

[[noreturn]] void usage_and_exit(const char *prog) {
    std::fprintf(stderr,
                 "Usage: %s --iface IFNAME --target slave=N,angle=DEG [--target ...] "
                 "[--duration-s S] [--ramp-s S] [--stagger-ms MS]\n"
                 "  --target is repeatable; at least one is required.\n"
                 "  angle is the ABSOLUTE target angle for that actuator (not a delta).\n"
                 "  --stagger-ms (default 100) delays each actuator's enable by this much\n"
                 "  from the previous one, to spread out simultaneous startup current draw.\n",
                 prog);
    std::exit(2);
}

TargetSpec parse_target(const std::string &spec, const char *prog) {
    TargetSpec t;
    bool have_slave = false;
    bool have_angle = false;
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
        } else if (key == "angle") {
            t.angle_deg = std::atof(value.c_str());
            have_angle = true;
        } else {
            std::fprintf(stderr, "error: unknown --target key '%s'\n", key.c_str());
            usage_and_exit(prog);
        }
    }
    if (!have_slave || !have_angle) usage_and_exit(prog);
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
        } else if (arg == "--stagger-ms") {
            args.stagger_ms = std::atof(next().c_str());
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

// request_operational_state() is a WHOLE-BUS outcome -- if it fails, a
// completely different slave than any target here could be why. Print
// every slave not already at OPERATIONAL, independent of any CiA-402
// interpretation. Mirrors the identical helper in zeroerr_move.cpp.
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

// Checks every actuator for a fault and, if any is present, pulses
// fault-reset on just those and waits up to 1s for it to clear. Returns
// false (caller should abort) only if a fault was present and did not
// clear. `context` names where in the sequence this is happening.
bool try_clear_faults(std::vector<zeroerr::ZeroErrActuator> &actuators, const std::vector<TargetSpec> &targets,
                       ethercat::SoemMaster &master, const char *context) {
    bool any_fault = false;
    for (std::size_t i = 0; i < actuators.size(); ++i) {
        if (actuators[i].has_fault()) {
            auto s = actuators[i].snapshot();
            std::printf("[%d] Fault detected (%s, error_code=0x%04X%s). Resetting...\n", targets[i].slave_index,
                        context, s.error_code, s.sto_active ? ", STO_ACTIVE" : "");
            actuators[i].fault_reset();
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
    // rest of this wait. Re-arm it periodically for whichever actuators
    // are still faulted, the same way request_operational_state() resends
    // its own one-shot request rather than sending it only once.
    constexpr int kRearmEveryNCycles = 40;  // ~200ms at the 5ms cycle
    auto next_wake = std::chrono::steady_clock::now();
    for (int i = 0; i < cycles_for(1.0) && !g_stop.load(); ++i) {
        if (i % kRearmEveryNCycles == 0) {
            for (auto &a : actuators) {
                if (a.has_fault()) {
                    a.fault_reset();
                }
            }
        }
        for (auto &a : actuators) {
            a.update();
        }
        master.send_receive();
        next_wake += kCycle;
        std::this_thread::sleep_until(next_wake);
    }
    bool all_clear = true;
    for (std::size_t i = 0; i < actuators.size(); ++i) {
        if (actuators[i].has_fault()) {
            std::fprintf(stderr, "error: slave [%d] fault did not clear (%s) -- aborting\n", targets[i].slave_index,
                         context);
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

    for (const auto &t : args.targets) {
        std::printf("Targeting slave [%d] name='%s' angle=%.2fdeg.\n", t.slave_index,
                     master.slave_name(t.slave_index).c_str(), t.angle_deg);
        zeroerr::configure_zeroerr_pdos(master, t.slave_index);
    }
    master.configure_pdos();

    if (!master.wait_for_safe_op()) {
        std::fprintf(stderr, "error: bus did not reach SAFE_OP -- aborting\n");
        print_unhealthy_slaves(master);
        return 1;
    }

    std::vector<zeroerr::ZeroErrActuator> actuators;
    actuators.reserve(args.targets.size());
    for (const auto &t : args.targets) {
        actuators.emplace_back(master, t.slave_index);
    }

    // One cycle of exchange so update() has a real statusword before we
    // decide whether a fault reset is needed.
    for (auto &a : actuators) {
        a.update();
    }
    master.send_receive();

    // Deliberately NOT aborting if this fails: AL state (SAFE_OP/
    // OPERATIONAL) and the CiA-402 application fault are separate layers
    // -- confirmed on real hardware that a ZeroErr eRob's 0xA000 ("master
    // offline") fault does not clear while still at SAFE_OP no matter how
    // many times fault_reset() is re-armed, plausibly because its
    // firmware only considers the master genuinely "online" once real
    // OPERATIONAL cyclic exchange is actually happening -- a
    // chicken-and-egg case if this checkpoint required clearing it first.
    // Try requesting OPERATIONAL anyway; the second checkpoint below,
    // once actually at OPERATIONAL, is the one that must succeed.
    if (!try_clear_faults(actuators, args.targets, master, "at SAFE_OP")) {
        std::fprintf(stderr, "warning: fault did not clear at SAFE_OP -- trying to reach OPERATIONAL anyway\n");
    }

    if (!master.request_operational_state()) {
        std::fprintf(stderr, "error: bus did not reach OPERATIONAL state -- aborting\n");
        print_unhealthy_slaves(master);
        return 1;
    }

    // Second check right at OPERATIONAL, before any enable command -- see
    // copley_move.cpp for why this is a distinct moment a fault can first
    // appear.
    for (auto &a : actuators) {
        a.update();
    }
    master.send_receive();
    if (!try_clear_faults(actuators, args.targets, master, "at OPERATIONAL, before any enable command")) {
        return 1;
    }

    std::printf("Enabling %zu actuator(s), staggered %.0fms apart...\n", actuators.size(), args.stagger_ms);
    const int stagger_cycles = std::max(0, static_cast<int>(args.stagger_ms / 1000.0 / kCycleSeconds));
    std::size_t next_to_enable = 0;
    std::vector<cia402::DriveState> last_states(actuators.size());
    for (std::size_t i = 0; i < actuators.size(); ++i) {
        last_states[i] = actuators[i].snapshot().state;
    }
    bool all_operational = false;
    auto next_wake = std::chrono::steady_clock::now();
    for (int i = 0; i < cycles_for(5.0) && !g_stop.load(); ++i) {
        // Enable one more actuator once its turn comes up, rather than all
        // at once -- each one's startup current inrush hits the shared
        // power rail as it transitions to OPERATION_ENABLED, and doing
        // that for several actuators in the same cycle can sag the bus
        // voltage enough to trip 0x3220 ("bus voltage undervoltage",
        // manual p.120) on all of them at once. Confirmed on real
        // hardware enabling 4 eRobs together.
        if (next_to_enable < actuators.size() && i >= static_cast<int>(next_to_enable) * stagger_cycles) {
            std::printf("  [%d] enabling\n", args.targets[next_to_enable].slave_index);
            actuators[next_to_enable].enable();
            ++next_to_enable;
        }
        for (auto &a : actuators) {
            a.update();
        }
        master.send_receive();
        bool all_ok = true;
        for (std::size_t j = 0; j < actuators.size(); ++j) {
            auto state = actuators[j].snapshot().state;
            if (state != last_states[j]) {
                std::printf("  [%d] state -> %s\n", args.targets[j].slave_index,
                            std::string(cia402::to_string(state)).c_str());
                last_states[j] = state;
            }
            if (j >= next_to_enable || !actuators[j].is_operational()) {
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
        std::fprintf(stderr, "error: not all actuators reached OPERATION_ENABLED within 5s -- aborting\n");
        for (std::size_t j = 0; j < actuators.size(); ++j) {
            if (!actuators[j].is_operational()) {
                std::fprintf(stderr, "  slave [%d] state=%s\n", args.targets[j].slave_index,
                             std::string(cia402::to_string(actuators[j].snapshot().state)).c_str());
            }
        }
        return 1;
    }

    std::vector<double> start_degs(actuators.size());
    // CSV (velocity) mode with the position loop closed here -- see
    // zeroerr/steer_position_controller.hpp -- rather than the CSP/Profile
    // Position modes this tool previously streamed a hand-ramped target
    // to. max_rate/max_accel per actuator are derived from --ramp-s so
    // each move still takes roughly that long, now via a real
    // accel-limited P loop instead of a fixed-shape linear ramp.
    std::vector<zeroerr::SteerPositionController> controllers;
    controllers.reserve(actuators.size());
    for (std::size_t j = 0; j < actuators.size(); ++j) {
        start_degs[j] = actuators[j].snapshot().position_deg;
        const double ramp_s = std::max(args.ramp_s, 0.05);
        const double max_rate_deg_s = std::max(std::abs(args.targets[j].angle_deg - start_degs[j]) / ramp_s, 1.0);
        controllers.emplace_back(max_rate_deg_s * M_PI / 180.0, (max_rate_deg_s / ramp_s) * M_PI / 180.0);
    }
    std::printf("Commanding %zu actuator(s) (ramping over %.2fs, holding %.2fs total, staggered start)...\n",
                actuators.size(), args.ramp_s, args.duration_s);

    // Actuator j starts being actively commanded j*stagger_cycles cycles
    // into this loop, not all at cycle 0 -- confirmed on real hardware:
    // commanding 4 actuators simultaneously tripped 0x3220 ("bus voltage
    // undervoltage") on all 4 at once, even though staggering the earlier
    // enable() call alone was not enough to prevent it. The loop runs
    // longer to compensate, so every actuator still gets its full
    // requested ramp_s + duration_s.
    const int total_cycles =
        cycles_for(args.duration_s) + static_cast<int>(actuators.size() - 1) * stagger_cycles;
    next_wake = std::chrono::steady_clock::now();
    for (int i = 0; i < total_cycles && !g_stop.load(); ++i) {
        for (std::size_t j = 0; j < actuators.size(); ++j) {
            actuators[j].update();
            int j_start_cycle = static_cast<int>(j) * stagger_cycles;
            if (i >= j_start_cycle) {
                const double actual_rad = actuators[j].snapshot().position_deg * M_PI / 180.0;
                const double target_rad = args.targets[j].angle_deg * M_PI / 180.0;
                const double velocity_rad_s = controllers[j].update(target_rad, actual_rad, kCycleSeconds);
                actuators[j].set_target_velocity_counts_per_s(zeroerr::deg_to_counts(velocity_rad_s * 180.0 / M_PI));
            }
        }
        master.send_receive();

        // Overwrite the same block in place instead of scrolling -- same
        // style as zeroerr_state/copley_state.
        if (i != 0) {
            std::printf("\033[%zuA", actuators.size());
        }
        for (std::size_t j = 0; j < actuators.size(); ++j) {
            auto s = actuators[j].snapshot();
            std::printf("  [%d] pos=%7.2f vel=%7.2f deg/s fault=%s\033[K\n", args.targets[j].slave_index,
                        s.position_deg, s.velocity_deg_per_s,
                        s.has_fault ? (s.sto_active ? "STO_ACTIVE" : "FAULT") : "-");
        }
        std::fflush(stdout);
        next_wake += kCycle;
        std::this_thread::sleep_until(next_wake);
    }

    std::printf("Disabling and closing bus.\n");
    for (auto &a : actuators) {
        a.disable();
    }
    next_wake = std::chrono::steady_clock::now();
    for (int i = 0; i < cycles_for(1.0); ++i) {
        for (auto &a : actuators) {
            a.update();
        }
        master.send_receive();
        next_wake += kCycle;
        std::this_thread::sleep_until(next_wake);
    }
    master.close();
    return 0;
}
