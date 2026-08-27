// Hands-on hardware bring-up tool: brings one axis (A or B) of a Copley
// BE2-090-20 through the DS402 enable sequence and commands it to a
// target velocity, ramping up and back down rather than stepping. Run
// with a human physically present, hand on the E-stop, wheel free to spin
// (off the ground / unloaded).
//
// Velocity is in raw encoder counts/s, NOT RPM or m/s -- there is no
// manual-confirmed encoder-resolution/gear-ratio for the ZL Tech drive
// motor yet (unlike the ZeroErr eRob's independently-confirmed 524288
// counts/rev), so no unit conversion is claimed here. Watch position/
// velocity_actual in the printed output to judge real-world speed until
// that's measured.
//
// Usage:
//   sudo ./copley_move --iface enp1s0 --axis a --velocity-counts-per-s 50000
//                       [--slave-index N] [--duration-s 5] [--ramp-s 1]
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
#include "copley/copley_axis.hpp"
#include "copley/copley_identity.hpp"
#include "ethercat/soem_master.hpp"

namespace {

std::atomic<bool> g_stop{false};
void on_sigint(int) { g_stop.store(true); }

struct Args {
    std::string iface;
    int slave_index = -1;  // -1 means auto-detect by identity
    copley::Axis axis = copley::Axis::A;
    std::int32_t velocity_counts_per_s = 0;
    double duration_s = 5.0;
    double ramp_s = -1.0;  // -1 means min(1.0, duration_s / 2)
};

[[noreturn]] void usage_and_exit(const char *prog) {
    std::fprintf(stderr,
                 "Usage: %s --iface IFNAME --axis {a|b} --velocity-counts-per-s N "
                 "[--slave-index N] [--duration-s S] [--ramp-s S]\n"
                 "  Velocity is raw encoder counts/s -- no RPM/m/s conversion is claimed\n"
                 "  (encoder resolution/gear ratio for this motor isn't confirmed yet).\n",
                 prog);
    std::exit(2);
}

Args parse_args(int argc, char **argv) {
    Args args;
    bool have_axis = false;
    bool have_velocity = false;
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
        } else if (arg == "--axis") {
            std::string v = next();
            if (v == "a" || v == "A") {
                args.axis = copley::Axis::A;
            } else if (v == "b" || v == "B") {
                args.axis = copley::Axis::B;
            } else {
                usage_and_exit(argv[0]);
            }
            have_axis = true;
        } else if (arg == "--velocity-counts-per-s") {
            args.velocity_counts_per_s = std::atoi(next().c_str());
            have_velocity = true;
        } else if (arg == "--duration-s") {
            args.duration_s = std::atof(next().c_str());
        } else if (arg == "--ramp-s") {
            args.ramp_s = std::atof(next().c_str());
        } else {
            usage_and_exit(argv[0]);
        }
    }
    if (args.iface.empty() || !have_axis || !have_velocity) usage_and_exit(argv[0]);
    if (args.ramp_s < 0.0) args.ramp_s = std::min(1.0, args.duration_s / 2.0);
    return args;
}

constexpr auto kCycle = std::chrono::microseconds(5000);
constexpr double kCycleSeconds = kCycle.count() / 1e6;

int cycles_for(double seconds) { return static_cast<int>(seconds / kCycleSeconds); }

const char *axis_name(copley::Axis axis) { return axis == copley::Axis::A ? "A" : "B"; }

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
        std::vector<int> matches = master.find_slaves_by_identity(copley::kVendorId, copley::kProductCode);
        if (matches.size() != 1) {
            std::fprintf(stderr,
                         "error: expected exactly one Copley BE2 on the bus, found %zu -- "
                         "pass --slave-index explicitly\n",
                         matches.size());
            for (int m : matches) std::fprintf(stderr, "  candidate: slave [%d] name='%s'\n", m,
                                                master.slave_name(m).c_str());
            return 1;
        }
        slave_index = matches.front();
    }
    std::printf("Targeting slave [%d] name='%s' axis=%s.\n", slave_index, master.slave_name(slave_index).c_str(),
                axis_name(args.axis));

    copley::configure_copley_pdos(master, slave_index);
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

    copley::CopleyAxis axis(master, slave_index, args.axis);

    // One cycle of exchange so update() has a real statusword before we
    // decide whether a fault reset is needed.
    axis.update();
    master.send_receive();

    if (axis.has_fault()) {
        auto s = axis.snapshot();
        std::printf("Pre-existing fault (error_code=0x%04X%s). Resetting (at SAFE_OP)...\n", s.error_code,
                    s.sto_active ? ", STO_ACTIVE" : "");
        axis.fault_reset();
        // Also clear the Copley-specific Latching Fault Status Register --
        // confirmed in the manual (p.69) as a genuinely separate mechanism
        // from the standard controlword Reset Fault bit.
        axis.clear_latching_faults();
        for (int i = 0; i < cycles_for(1.0) && !g_stop.load(); ++i) {
            axis.update();
            master.send_receive();
            std::this_thread::sleep_for(kCycle);
        }
        if (axis.has_fault()) {
            std::fprintf(stderr, "error: fault did not clear -- aborting\n");
            return 1;
        }
    }

    if (!master.request_operational_state()) {
        std::fprintf(stderr, "error: bus did not reach OPERATIONAL state -- aborting\n");
        print_unhealthy_slaves(master);
        return 1;
    }

    axis.enable();
    std::printf("Enabling axis %s...\n", axis_name(args.axis));
    cia402::DriveState last_state = axis.snapshot().state;
    bool reached_operational = false;
    for (int i = 0; i < cycles_for(5.0) && !g_stop.load(); ++i) {
        axis.update();
        master.send_receive();
        auto state = axis.snapshot().state;
        if (state != last_state) {
            std::printf("  state -> %s\n", std::string(cia402::to_string(state)).c_str());
            last_state = state;
        }
        if (axis.is_operational()) {
            reached_operational = true;
            break;
        }
        std::this_thread::sleep_for(kCycle);
    }
    if (!reached_operational) {
        std::fprintf(stderr, "error: did not reach OPERATION_ENABLED within 5s -- aborting\n");
        return 1;
    }

    std::printf("Commanding velocity 0 -> %d counts/s (ramping over %.2fs, holding %.2fs total)...\n",
                args.velocity_counts_per_s, args.ramp_s, args.duration_s);

    const int total_cycles = cycles_for(args.duration_s);
    for (int i = 0; i < total_cycles && !g_stop.load(); ++i) {
        double elapsed_s = i * kCycleSeconds;
        double fraction = args.ramp_s > 0.0 ? std::min(1.0, elapsed_s / args.ramp_s) : 1.0;
        auto command = static_cast<std::int32_t>(args.velocity_counts_per_s * fraction);

        axis.update();
        axis.set_target_velocity_counts_per_s(command);
        master.send_receive();

        auto s = axis.snapshot();
        std::printf("  cmd=%8d vel=%8d pos=%10d foll_err=%7d torque=%6d fault=%s\n", command,
                    s.velocity_actual_counts_per_s, s.position_actual_counts, s.following_error_counts,
                    s.torque_actual_raw, s.has_fault ? (s.sto_active ? "STO_ACTIVE" : "FAULT") : "-");
        std::this_thread::sleep_for(kCycle);
    }

    std::printf("Ramping down to zero before disabling...\n");
    const int ramp_down_cycles = cycles_for(args.ramp_s > 0.0 ? args.ramp_s : 1.0);
    for (int i = 0; i < ramp_down_cycles; ++i) {
        double fraction = 1.0 - static_cast<double>(i) / ramp_down_cycles;
        auto command = static_cast<std::int32_t>(args.velocity_counts_per_s * fraction);
        axis.update();
        axis.set_target_velocity_counts_per_s(command);
        master.send_receive();
        std::this_thread::sleep_for(kCycle);
    }

    std::printf("Disabling and closing bus.\n");
    axis.disable();
    for (int i = 0; i < cycles_for(1.0); ++i) {
        axis.update();
        master.send_receive();
        std::this_thread::sleep_for(kCycle);
    }
    master.close();
    return 0;
}
