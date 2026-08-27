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

    // One cycle of exchange so update() has a real statusword before we
    // decide whether a fault reset is needed.
    actuator.update();
    master.send_receive();

    if (actuator.has_fault()) {
        auto s = actuator.snapshot();
        std::printf("Pre-existing fault (error_code=0x%04X%s). Resetting (at SAFE_OP)...\n", s.error_code,
                    s.sto_active ? ", STO_ACTIVE" : "");
        actuator.fault_reset();
        auto next_wake = std::chrono::steady_clock::now();
        for (int i = 0; i < cycles_for(1.0) && !g_stop.load(); ++i) {
            actuator.update();
            master.send_receive();
            next_wake += kCycle;
            std::this_thread::sleep_until(next_wake);
        }
        if (actuator.has_fault()) {
            std::fprintf(stderr, "error: fault did not clear -- aborting\n");
            return 1;
        }
    }

    if (!master.request_operational_state()) {
        std::fprintf(stderr, "error: bus did not reach OPERATIONAL state -- aborting\n");
        print_unhealthy_slaves(master);
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
    std::printf("Commanding move %.2f -> %.2f deg (ramping over %.2fs, holding %.2fs total)...\n", start_deg,
                target_deg, args.ramp_s, args.duration_s);

    const int total_cycles = cycles_for(args.duration_s);
    next_wake = std::chrono::steady_clock::now();
    for (int i = 0; i < total_cycles && !g_stop.load(); ++i) {
        double elapsed_s = i * kCycleSeconds;
        double fraction = args.ramp_s > 0.0 ? std::min(1.0, elapsed_s / args.ramp_s) : 1.0;
        double command_deg = start_deg + (target_deg - start_deg) * fraction;

        actuator.update();
        actuator.set_target_angle_deg(command_deg);
        master.send_receive();

        // One line per 5ms cycle would flood the terminal (2000+ lines for
        // a 10s run) -- overwrite the same line in place instead, same as
        // zeroerr_state/copley_state do.
        auto s = actuator.snapshot();
        if (i != 0) {
            std::printf("\033[1A");
        }
        std::printf("  cmd=%7.2f pos=%7.2f vel=%7.2f deg/s fault=%s\033[K\n", command_deg, s.position_deg,
                    s.velocity_deg_per_s, s.has_fault ? (s.sto_active ? "STO_ACTIVE" : "FAULT") : "-");
        std::fflush(stdout);
        next_wake += kCycle;
        std::this_thread::sleep_until(next_wake);
    }

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
