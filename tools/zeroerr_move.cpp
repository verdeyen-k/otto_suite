// Hands-on hardware bring-up tool: brings exactly one ZeroErr eRob through
// the DS402 enable sequence and commands it to an absolute target angle,
// ramping linearly rather than stepping instantly. Run with a human
// physically present, hand on the E-stop, mechanism free to rotate.
//
// Usage:
//   sudo ./zeroerr_move --iface enp1s0 --angle-deg 10 [--slave-index N]
//                        [--duration-s 5] [--ramp-s 1]
#include <algorithm>
#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstdlib>
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

struct Args {
    std::string iface;
    int slave_index = -1;  // -1 means auto-detect by identity
    double angle_deg = 10.0;
    double duration_s = 5.0;
    double ramp_s = -1.0;  // -1 means min(1.0, duration_s / 2)
};

[[noreturn]] void usage_and_exit(const char *prog) {
    std::fprintf(stderr,
                 "Usage: %s --iface IFNAME --angle-deg DEG [--slave-index N] "
                 "[--duration-s S] [--ramp-s S]\n"
                 "  --angle-deg is the ABSOLUTE target angle to move to (not a delta).\n",
                 prog);
    std::exit(2);
}

Args parse_args(int argc, char **argv) {
    Args args;
    bool have_angle = false;
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        auto next = [&]() -> std::string {
            if (i + 1 >= argc) usage_and_exit(argv[0]);
            return argv[++i];
        };
        if (arg == "--iface") {
            args.iface = next();
        } else if (arg == "--slave-index") {
            args.slave_index = std::atoi(next().c_str());
        } else if (arg == "--angle-deg") {
            args.angle_deg = std::atof(next().c_str());
            have_angle = true;
        } else if (arg == "--duration-s") {
            args.duration_s = std::atof(next().c_str());
        } else if (arg == "--ramp-s") {
            args.ramp_s = std::atof(next().c_str());
        } else {
            usage_and_exit(argv[0]);
        }
    }
    if (args.iface.empty() || !have_angle) usage_and_exit(argv[0]);
    if (args.ramp_s < 0.0) args.ramp_s = std::min(1.0, args.duration_s / 2.0);
    return args;
}

constexpr auto kCycle = std::chrono::microseconds(5000);
constexpr double kCycleSeconds = kCycle.count() / 1e6;

int cycles_for(double seconds) { return static_cast<int>(seconds / kCycleSeconds); }

// request_operational_state() is a WHOLE-BUS outcome (state is written to
// slave 0, broadcast) -- if it fails, a completely different slave than
// the one this tool is targeting could be why. Print every slave not
// already at OPERATIONAL, independent of any CiA-402 interpretation.
void print_unhealthy_slaves(ethercat::SoemMaster &master) {
    // A slave can sit at SAFE_OP indefinitely with no AL error at all if
    // it's simply not fully acknowledging the cyclic exchange -- compare
    // one more actual working counter against what the IO mapping
    // expects (same "Calculated workcounter" SOEM's own samples print).
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

    int slave_index = args.slave_index;
    if (slave_index < 0) {
        std::vector<int> matches = master.find_slaves_by_identity(zeroerr::kVendorId, zeroerr::kProductCode);
        if (matches.size() != 1) {
            std::fprintf(stderr,
                         "error: expected exactly one ZeroErr eRob on the bus, found %zu -- "
                         "pass --slave-index explicitly\n",
                         matches.size());
            for (int m : matches) std::fprintf(stderr, "  candidate: slave [%d] name='%s'\n", m,
                                                master.slave_name(m).c_str());
            return 1;
        }
        slave_index = matches.front();
    }
    std::printf("Targeting slave [%d] name='%s'.\n", slave_index, master.slave_name(slave_index).c_str());

    zeroerr::configure_zeroerr_pdos(master, slave_index);
    master.configure_pdos();

    // Fault-reset the CiA402 way BEFORE requesting full OPERATIONAL, not
    // after: a fault-reset controlword pulse may only take effect while
    // still at PRE-OP/SAFE-OP on this hardware, and a slave whose own
    // application-layer fault is what's blocking the AL-level SAFE_OP ->
    // OPERATIONAL transition would otherwise be a chicken-and-egg
    // deadlock -- OP required before we ever try clearing the fault that
    // itself prevents reaching OP. Confirmed necessary in practice: a
    // straggler eRob refusing OP with a clean working counter and no AL
    // error traced back to exactly this ordering.
    if (!master.wait_for_safe_op()) {
        std::fprintf(stderr, "error: bus did not reach SAFE_OP -- aborting\n");
        print_unhealthy_slaves(master);
        return 1;
    }

    zeroerr::ZeroErrActuator actuator(master, slave_index);

    // Checks for a fault and, if present, re-arms the standard controlword
    // Reset Fault pulse periodically (fault_reset() only sends a single
    // rising-edge pulse -- if that one doesn't land, or the fault
    // re-triggers right as it's processed, nothing resends it otherwise)
    // for up to 1s. `context` names where in the sequence this is
    // happening, for the printed message. Returns true if no fault was
    // present or it cleared, false if it was present and never did.
    auto try_clear_fault = [&](const char *context) -> bool {
        if (!actuator.has_fault()) {
            return true;
        }
        auto s = actuator.snapshot();
        std::printf("Fault detected (%s, error_code=0x%04X%s). Resetting...\n", context, s.error_code,
                    s.sto_active ? ", STO_ACTIVE" : "");
        actuator.fault_reset();
        constexpr int kRearmEveryNCycles = 40;  // ~200ms at the 5ms cycle
        auto next_wake = std::chrono::steady_clock::now();
        for (int i = 0; i < cycles_for(1.0) && !g_stop.load(); ++i) {
            if (i % kRearmEveryNCycles == 0 && actuator.has_fault()) {
                actuator.fault_reset();
            }
            actuator.update();
            master.send_receive();
            next_wake += kCycle;
            std::this_thread::sleep_until(next_wake);
        }
        return !actuator.has_fault();
    };

    // One cycle of exchange so update() has a real statusword before we
    // decide whether a fault reset is needed.
    actuator.update();
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
    if (!try_clear_fault("at SAFE_OP")) {
        std::fprintf(stderr, "warning: fault did not clear at SAFE_OP -- trying to reach OPERATIONAL anyway\n");
    }

    if (!master.request_operational_state()) {
        std::fprintf(stderr, "error: bus did not reach OPERATIONAL state -- aborting\n");
        print_unhealthy_slaves(master);
        return 1;
    }

    // Second check right at OPERATIONAL, before any enable command -- a
    // fault arising from the OPERATIONAL transition itself (as opposed to
    // anything commanded afterward) would otherwise be invisible until the
    // enable loop's first iteration. See copley_move.cpp for the same
    // reasoning.
    actuator.update();
    master.send_receive();
    if (!try_clear_fault("at OPERATIONAL, before any enable command")) {
        std::fprintf(stderr, "error: fault did not clear -- aborting\n");
        return 1;
    }

    actuator.enable();
    std::printf("Enabling...\n");
    cia402::DriveState last_state = actuator.snapshot().state;
    bool reached_operational = false;
    auto next_wake = std::chrono::steady_clock::now();
    for (int i = 0; i < cycles_for(5.0) && !g_stop.load(); ++i) {
        actuator.update();
        master.send_receive();
        auto state = actuator.snapshot().state;
        if (state != last_state) {
            std::printf("  state -> %s\n", std::string(cia402::to_string(state)).c_str());
            last_state = state;
        }
        if (actuator.is_operational()) {
            reached_operational = true;
            break;
        }
        next_wake += kCycle;
        std::this_thread::sleep_until(next_wake);
    }
    if (!reached_operational) {
        std::fprintf(stderr, "error: did not reach OPERATION_ENABLED within 5s -- aborting\n");
        return 1;
    }

    const double start_deg = actuator.snapshot().position_deg;
    const double target_deg = args.angle_deg;
    // CSV (velocity) mode with the position loop closed here -- see
    // zeroerr/steer_position_controller.hpp -- rather than the CSP/Profile
    // Position modes this tool previously streamed a hand-ramped target
    // to. max_rate/max_accel are derived from --ramp-s so the move still
    // takes roughly that long, now via a real accel-limited P loop instead
    // of a fixed-shape linear ramp.
    const double ramp_s = std::max(args.ramp_s, 0.05);
    const double max_rate_deg_s = std::max(std::abs(target_deg - start_deg) / ramp_s, 1.0);
    const double max_accel_deg_s2 = max_rate_deg_s / ramp_s;
    zeroerr::SteerPositionController controller(max_rate_deg_s * M_PI / 180.0, max_accel_deg_s2 * M_PI / 180.0);
    std::printf("Commanding move %.2f -> %.2f deg (max_rate=%.1fdeg/s over ~%.2fs, holding %.2fs total)...\n",
                start_deg, target_deg, max_rate_deg_s, args.ramp_s, args.duration_s);

    const int total_cycles = cycles_for(args.duration_s);
    next_wake = std::chrono::steady_clock::now();
    for (int i = 0; i < total_cycles && !g_stop.load(); ++i) {
        actuator.update();
        const double actual_rad = actuator.snapshot().position_deg * M_PI / 180.0;
        const double velocity_rad_s = controller.update(target_deg * M_PI / 180.0, actual_rad, kCycleSeconds);
        actuator.set_target_velocity_counts_per_s(zeroerr::deg_to_counts(velocity_rad_s * 180.0 / M_PI));
        master.send_receive();

        // One line per 5ms cycle would flood the terminal (2000+ lines for
        // a 10s run) -- overwrite the same line in place instead, same as
        // zeroerr_state/copley_state do.
        auto s = actuator.snapshot();
        if (i != 0) {
            std::printf("\033[1A");
        }
        std::printf("  cmd_vel=%7.2f pos=%7.2f vel=%7.2f deg/s fault=%s\033[K\n",
                    velocity_rad_s * 180.0 / M_PI, s.position_deg, s.velocity_deg_per_s,
                    s.has_fault ? (s.sto_active ? "STO_ACTIVE" : "FAULT") : "-");
        std::fflush(stdout);
        next_wake += kCycle;
        std::this_thread::sleep_until(next_wake);
    }
    actuator.set_target_velocity_counts_per_s(0);

    std::printf("Disabling and closing bus.\n");
    actuator.disable();
    next_wake = std::chrono::steady_clock::now();
    for (int i = 0; i < cycles_for(1.0); ++i) {
        actuator.update();
        master.send_receive();
        next_wake += kCycle;
        std::this_thread::sleep_until(next_wake);
    }
    master.close();
    return 0;
}
