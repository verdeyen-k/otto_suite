// Passive diagnostic tool: connects to Copley BE2-090-20 slave(s) on the
// given EtherCAT interface and continuously prints every field this
// driver's PDO/SDO mapping exposes for BOTH axes (A and B are always
// mapped together, see copley_axis.hpp). Never sends an enable or
// fault-reset transition -- safe to run with zero setup beyond the
// drive(s) being wired and powered.
//
// Usage: sudo ./copley_state --iface enp1s0 [--slave-index N] [--rate-hz 5]
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
    double rate_hz = 5.0;
};

[[noreturn]] void usage_and_exit(const char *prog) {
    std::fprintf(stderr,
                 "Usage: %s --iface IFNAME [--slave-index N] [--rate-hz HZ]\n"
                 "  Passive: connects, reads state for both axes, never enables or moves anything.\n",
                 prog);
    std::exit(2);
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
        } else if (arg == "--slave-index") {
            args.slave_index = std::atoi(next().c_str());
        } else if (arg == "--rate-hz") {
            args.rate_hz = std::atof(next().c_str());
        } else {
            usage_and_exit(argv[0]);
        }
    }
    if (args.iface.empty()) usage_and_exit(argv[0]);
    return args;
}

const char *fault_flags(const copley::StateSnapshot &s) {
    if (!s.has_fault) return "-";
    if (s.sto_active) return "STO_ACTIVE";
    return "FAULT";
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

    std::vector<int> copley_slaves = master.find_slaves_by_identity(copley::kVendorId, copley::kProductCode);
    std::vector<int> target_slaves;
    if (args.slave_index >= 0) {
        target_slaves = {args.slave_index};
    } else if (copley_slaves.empty()) {
        std::fprintf(stderr, "error: no Copley BE2 slaves found on the bus (vendor=0x%08X product=0x%08X)\n",
                     copley::kVendorId, copley::kProductCode);
        return 1;
    } else {
        target_slaves = copley_slaves;
    }

    for (int idx : target_slaves) {
        std::printf("Monitoring slave [%d] name='%s' (axes A and B)\n", idx, master.slave_name(idx).c_str());
        copley::configure_copley_pdos(master, idx);
    }
    master.configure_pdos();

    // Only SAFE_OP is actually required for input PDO data to be valid --
    // see the same reasoning in zeroerr_state.cpp. Full OP is attempted
    // but not required, so a bus stuck below OP for some other reason
    // still shows explainable state instead of a bare abort.
    if (!master.wait_for_safe_op()) {
        std::fprintf(stderr, "error: bus did not reach SAFE_OP -- no input PDO data would be valid\n");
        return 1;
    }
    bool reached_op = master.request_operational_state();
    std::printf("Bus reached SAFE_OP%s. Press Ctrl+C to stop.\n\n", reached_op ? " and OPERATIONAL" : "");
    if (!reached_op) {
        std::printf("NOTE: did not reach OPERATIONAL -- state below is still live (SAFE_OP inputs are valid),\n"
                     "      but check for a fault/error_code explaining why OP wasn't reached.\n\n");
    }

    struct Target {
        int slave_index;
        copley::Axis axis;
        copley::CopleyAxis handle;
    };
    std::vector<Target> targets;
    targets.reserve(target_slaves.size() * 2);
    for (int idx : target_slaves) {
        targets.push_back({idx, copley::Axis::A, copley::CopleyAxis(master, idx, copley::Axis::A)});
        targets.push_back({idx, copley::Axis::B, copley::CopleyAxis(master, idx, copley::Axis::B)});
    }

    const auto cycle = std::chrono::microseconds(5000);
    const int print_every =
        args.rate_hz > 0 ? std::max(1, static_cast<int>(1.0 / args.rate_hz / (cycle.count() / 1e6))) : 1;
    int cycle_count = 0;

    while (!g_stop.load()) {
        for (auto &t : targets) {
            t.handle.update();
        }
        master.send_receive();

        if (cycle_count % print_every == 0) {
            if (cycle_count != 0) {
                std::printf("\033[%zuA", targets.size());
            }
            for (auto &t : targets) {
                const auto s = t.handle.snapshot();
                // Live, enable-independent SDO read of the STO input --
                // separate from the fault/error_code fields, which are
                // only populated on a fault edge and may need an actual
                // enable attempt to latch. See copley_identity.hpp. A
                // failed read (object unsupported on this drive, or a
                // mailbox timeout) is reported explicitly rather than
                // shown as a misleading all-zero value.
                auto safety = t.handle.read_safety_circuit_status();
                char safety_field[48];
                if (safety.has_value()) {
                    bool blocking = (*safety & copley::kSafetyCircuitInput0Blocking) != 0;
                    std::snprintf(safety_field, sizeof(safety_field), "safety=0x%08X%s", *safety,
                                  blocking ? " STO_INPUT_BLOCKING" : "");
                } else {
                    std::snprintf(safety_field, sizeof(safety_field), "safety=READ_FAILED(unsupported?)");
                }
                std::printf(
                    "[%d/%c] state=%-22s sw=0x%04X cw=0x%04X vel=%8d(cmd=%8d) pos=%10d foll_err=%7d "
                    "torque=%6d err=0x%04X fault=%s %s\033[K\n",
                    t.slave_index, t.axis == copley::Axis::A ? 'A' : 'B',
                    std::string(cia402::to_string(s.state)).c_str(), s.statusword_raw, s.controlword_raw,
                    s.velocity_actual_counts_per_s, s.commanded_velocity_counts_per_s, s.position_actual_counts,
                    s.following_error_counts, s.torque_actual_raw, s.error_code, fault_flags(s), safety_field);
            }
            std::fflush(stdout);
        }
        ++cycle_count;
        std::this_thread::sleep_for(cycle);
    }

    std::printf("\nClosing bus.\n");
    master.close();
    return 0;
}
