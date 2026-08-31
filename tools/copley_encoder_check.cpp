// One-shot diagnostic: reads the actual configured Motor Encoder
// Counts/Rev (object 0x2383, sub-index 23) from every Copley BE2 drive
// axis on the bus and prints it. Pure SDO read -- no PDO mapping or
// enable needed, since mailbox comms are already up once scan() reaches
// PRE-OP. Exists because config/robot_constants.yaml's
// drive_encoder_counts_per_rev (524288) was "confirmed by you, not
// derived from a manual" per SESSION_NOTES.md, and real-hardware driving
// suggests it may not match what's actually configured on the drive.
//
// Usage: sudo ./copley_encoder_check --iface enp1s0
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <optional>
#include <string>
#include <vector>

#include "copley/copley_identity.hpp"
#include "ethercat/soem_master.hpp"

namespace {

// Manual p.89: "MOTOR ENCODER COUNTS/REV", index 0x2383, sub-index 23
// (decimal -- confirmed against the manual's own sequential decimal
// sub-index table for this object, not to be read as 0x23), UNSIGNED32,
// counts/rev, RW, not PDO-mappable.
constexpr std::uint16_t kMotorEncoderCountsPerRevIndex = 0x2383;
constexpr std::uint8_t kMotorEncoderCountsPerRevSubindex = 23;

[[noreturn]] void usage_and_exit(const char *prog) {
    std::fprintf(stderr, "Usage: %s --iface IFNAME\n", prog);
    std::exit(2);
}

std::string parse_iface(int argc, char **argv) {
    std::string iface;
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--iface" && i + 1 < argc) {
            iface = argv[++i];
        } else {
            usage_and_exit(argv[0]);
        }
    }
    if (iface.empty()) usage_and_exit(argv[0]);
    return iface;
}

std::optional<std::uint32_t> read_encoder_counts_per_rev(const ethercat::SoemMaster &master, int slave_index,
                                                          copley::Axis axis) {
    std::uint32_t value = 0;
    const std::uint16_t index = kMotorEncoderCountsPerRevIndex + copley::axis_object_offset(axis);
    int wkc = master.sdo_read(slave_index, index, kMotorEncoderCountsPerRevSubindex, &value, sizeof(value));
    if (wkc <= 0) return std::nullopt;
    return value;
}

void print_result(const char *label, std::optional<std::uint32_t> counts_per_rev) {
    if (counts_per_rev.has_value()) {
        std::printf("  %s %u counts/rev\n", label, *counts_per_rev);
    } else {
        std::printf("  %s READ_FAILED\n", label);
    }
}

}  // namespace

int main(int argc, char **argv) {
    std::string iface = parse_iface(argc, argv);

    ethercat::SoemMaster master(iface);
    int slave_count;
    try {
        slave_count = master.scan();
    } catch (const std::exception &e) {
        std::fprintf(stderr, "error: %s\n", e.what());
        return 1;
    }
    std::printf("Found %d slave(s) on '%s'.\n\n", slave_count, iface.c_str());

    std::vector<int> drive_slaves = master.find_slaves_by_identity(copley::kVendorId, copley::kProductCode);
    std::printf("Drive axes (Copley BE2, object 0x%04X sub-index %d, Motor Encoder Counts/Rev):\n",
                kMotorEncoderCountsPerRevIndex, kMotorEncoderCountsPerRevSubindex);
    if (drive_slaves.empty()) {
        std::printf("  (none found)\n");
    }
    for (int idx : drive_slaves) {
        char label_a[32];
        std::snprintf(label_a, sizeof(label_a), "[slave %d / A]", idx);
        print_result(label_a, read_encoder_counts_per_rev(master, idx, copley::Axis::A));
        char label_b[32];
        std::snprintf(label_b, sizeof(label_b), "[slave %d / B]", idx);
        print_result(label_b, read_encoder_counts_per_rev(master, idx, copley::Axis::B));
    }

    std::printf("\nconfig/robot_constants.yaml currently has drive_encoder_counts_per_rev: 524288 -- if the "
                "values above disagree, update that to match.\n");

    master.close();
    return 0;
}
