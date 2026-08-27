// Passive diagnostic tool: connects to a ZeroErr eRob on the given
// EtherCAT interface and continuously prints every field this driver's
// extended PDO/SDO mapping exposes. Never sends an enable or fault-reset
// transition -- safe to run with zero setup beyond the actuator being
// wired and powered.
//
// Usage: sudo ./zeroerr_state --iface enp1s0 [--slave-index N] [--rate-hz 5]
#include <algorithm>
#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
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
    double rate_hz = 5.0;
};

[[noreturn]] void usage_and_exit(const char *prog) {
    std::fprintf(stderr,
                 "Usage: %s --iface IFNAME [--slave-index N] [--rate-hz HZ]\n"
                 "  Passive: connects, reads state, never enables or moves anything.\n",
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

const char *fault_flags(const zeroerr::StateSnapshot &s) {
    if (!s.has_fault) return "-";
    if (s.error_code_read_failed) return "FAULT(err read FAILED)";
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

    std::vector<int> zeroerr_slaves = master.find_slaves_by_identity(zeroerr::kVendorId, zeroerr::kProductCode);
    std::vector<int> target_slaves;
    if (args.slave_index >= 0) {
        target_slaves = {args.slave_index};
    } else if (zeroerr_slaves.empty()) {
        std::fprintf(stderr, "error: no ZeroErr eRob slaves found on the bus (vendor=0x%08X product=0x%08X)\n",
                     zeroerr::kVendorId, zeroerr::kProductCode);
        return 1;
    } else {
        target_slaves = zeroerr_slaves;
    }

    for (int idx : target_slaves) {
        std::printf("Monitoring slave [%d] name='%s'\n", idx, master.slave_name(idx).c_str());
        zeroerr::configure_zeroerr_pdos(master, idx);
    }
    master.configure_pdos();

    // SAFE_OP is the actual minimum for input PDO data (statusword,
    // position, etc.) to be valid -- if the bus can't even reach that,
    // there's nothing meaningful to print. Full OPERATIONAL is attempted
    // too but not required: this tool never applies outputs, so a bus
    // stuck below OP for some other reason (a fault, a watchdog issue)
    // should still show its state instead of a bare abort.
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

    std::vector<zeroerr::ZeroErrActuator> actuators;
    actuators.reserve(target_slaves.size());
    for (int idx : target_slaves) {
        actuators.emplace_back(master, idx);
    }

    const auto cycle = std::chrono::microseconds(5000);
    const int print_every =
        args.rate_hz > 0 ? std::max(1, static_cast<int>(1.0 / args.rate_hz / (cycle.count() / 1e6))) : 1;
    int cycle_count = 0;
    auto next_wake = std::chrono::steady_clock::now();

    while (!g_stop.load()) {
        for (auto &actuator : actuators) {
            actuator.update();
        }
        master.send_receive();

        if (cycle_count % print_every == 0) {
            // Move the cursor back up over the previous block (two lines
            // per actuator) so each print overwrites in place instead of
            // scrolling -- skip on the very first print, nothing to
            // overwrite yet.
            constexpr int kLinesPerActuator = 2;
            if (cycle_count != 0) {
                std::printf("\033[%zuA", actuators.size() * kLinesPerActuator);
            }
            for (std::size_t i = 0; i < actuators.size(); ++i) {
                const auto s = actuators[i].snapshot();
                // s.error_code is only ever captured reactively, on a
                // CiA-402 statusword FAULT edge -- a slave stuck at the
                // EtherCAT AL layer (e.g. never leaving SAFE_OP) without
                // ever reporting a DS402-level FAULT would otherwise never
                // have its error register checked at all. Read it live,
                // unconditionally, every print cycle instead.
                auto live_err = actuators[i].read_error_code_live();
                char live_err_field[32];
                if (live_err.has_value()) {
                    std::snprintf(live_err_field, sizeof(live_err_field), "0x%04X", *live_err);
                } else {
                    std::snprintf(live_err_field, sizeof(live_err_field), "READ_FAILED");
                }
                std::printf(
                    "[%d] state=%-22s sw=0x%04X cw=0x%04X pos=%9d(%8.2f deg) vel=%9d(%8.2f deg/s) "
                    "eff=%6d din=0x%08X dout=0x%08X mode_disp=%u err=0x%04X fault=%s err_live=%s\033[K\n",
                    target_slaves[i], std::string(cia402::to_string(s.state)).c_str(), s.statusword_raw,
                    s.controlword_raw, s.position_counts, s.position_deg, s.velocity_actual_counts_per_s,
                    s.velocity_deg_per_s, s.effort_actual_raw, s.digital_inputs_raw, s.digital_outputs_raw,
                    s.mode_of_operation_display, s.error_code, fault_flags(s), live_err_field);

                // DC sync diagnostics (manual p.42, Table 4-8) -- the
                // slave's own record of whether the master is actually
                // meeting its cyclic timing, direct evidence for whether
                // an RT kernel would help versus guessing from symptoms.
                auto missed_sm2 = actuators[i].read_sm_event_missed(/*outputs=*/true);
                auto missed_sm3 = actuators[i].read_sm_event_missed(/*outputs=*/false);
                auto sync_err_sm2 = actuators[i].read_sync_error(/*outputs=*/true);
                auto sync_err_sm3 = actuators[i].read_sync_error(/*outputs=*/false);
                char missed_field[64];
                std::snprintf(missed_field, sizeof(missed_field), "sm_event_missed out=%s in=%s",
                              missed_sm2.has_value() ? std::to_string(*missed_sm2).c_str() : "READ_FAILED",
                              missed_sm3.has_value() ? std::to_string(*missed_sm3).c_str() : "READ_FAILED");
                char sync_err_field[48];
                std::snprintf(sync_err_field, sizeof(sync_err_field), "sync_error out=%s in=%s",
                              sync_err_sm2.has_value() ? (*sync_err_sm2 ? "YES" : "no") : "READ_FAILED",
                              sync_err_sm3.has_value() ? (*sync_err_sm3 ? "YES" : "no") : "READ_FAILED");
                std::printf("      %s %s\033[K\n", missed_field, sync_err_field);
            }
            std::fflush(stdout);
        }
        ++cycle_count;
        next_wake += cycle;
        std::this_thread::sleep_until(next_wake);
    }

    std::printf("\nClosing bus.\n");
    master.close();
    return 0;
}
