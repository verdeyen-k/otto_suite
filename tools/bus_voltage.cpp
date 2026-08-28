// One-shot diagnostic: reads and prints the DC bus voltage of every
// actuator axis on the bus (4 ZeroErr eRob steering actuators, 4 Copley
// BE2 drive axes across 2 physical dual-axis drives) and exits. Pure SDO
// (mailbox) reads only -- no PDO mapping, no CiA-402 enable, nothing
// moves. Mailbox communication is already up once scan() brings slaves to
// PRE-OP (SOEM's ecx_config_init does this), so this doesn't need
// configure_pdos()/wait_for_safe_op() at all, unlike every other tool
// here that reads live PDO fields.
//
// Usage: sudo ./bus_voltage --iface enp1s0
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <optional>
#include <string>
#include <vector>

#include "copley/copley_identity.hpp"
#include "ethercat/soem_master.hpp"
#include "zeroerr/zeroerr_identity.hpp"

namespace {

// eRob CANopen/EtherCAT User Manual sec 8.2.89: 0x6079:00 "DC link circuit
// voltage", UDINT, millivolts, RO -- the standard CiA-402 object.
constexpr std::uint16_t kZeroErrBusVoltageIndex = 0x6079;

// Copley CANopen and EtherCAT Programmer's Manual p.105, sec 5.6: 0x2201
// "High Voltage Reference" (a.k.a. Bus Voltage), INTEGER16, 0.1V units, RO.
// Manufacturer-specific range (0x2000-0x27FF), so it's offset by
// axis_object_offset() the same way as every other per-axis Copley object
// in this codebase (see copley_identity.hpp).
constexpr std::uint16_t kCopleyBusVoltageIndex = 0x2201;

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

std::optional<double> read_zeroerr_voltage(const ethercat::SoemMaster &master, int slave_index) {
    std::uint32_t raw_mv = 0;
    int wkc = master.sdo_read(slave_index, kZeroErrBusVoltageIndex, 0, &raw_mv, sizeof(raw_mv));
    if (wkc <= 0) return std::nullopt;
    return raw_mv / 1000.0;
}

std::optional<double> read_copley_voltage(const ethercat::SoemMaster &master, int slave_index, copley::Axis axis) {
    std::int16_t raw_decivolts = 0;
    const std::uint16_t index = kCopleyBusVoltageIndex + copley::axis_object_offset(axis);
    int wkc = master.sdo_read(slave_index, index, 0, &raw_decivolts, sizeof(raw_decivolts));
    if (wkc <= 0) return std::nullopt;
    return raw_decivolts * 0.1;
}

void print_voltage(const char *label, std::optional<double> volts) {
    if (volts.has_value()) {
        std::printf("  %s %.1f V\n", label, *volts);
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

    std::vector<int> steer_slaves = master.find_slaves_by_identity(zeroerr::kVendorId, zeroerr::kProductCode);
    std::printf("Steer actuators (ZeroErr eRob, object 0x%04X DC link circuit voltage):\n",
                kZeroErrBusVoltageIndex);
    if (steer_slaves.empty()) {
        std::printf("  (none found)\n");
    }
    for (int idx : steer_slaves) {
        char label[32];
        std::snprintf(label, sizeof(label), "[slave %d]", idx);
        print_voltage(label, read_zeroerr_voltage(master, idx));
    }

    std::vector<int> drive_slaves = master.find_slaves_by_identity(copley::kVendorId, copley::kProductCode);
    std::printf("\nDrive axes (Copley BE2, object 0x%04X High Voltage Reference):\n", kCopleyBusVoltageIndex);
    if (drive_slaves.empty()) {
        std::printf("  (none found)\n");
    }
    for (int idx : drive_slaves) {
        char label_a[32];
        std::snprintf(label_a, sizeof(label_a), "[slave %d / A]", idx);
        print_voltage(label_a, read_copley_voltage(master, idx, copley::Axis::A));
        char label_b[32];
        std::snprintf(label_b, sizeof(label_b), "[slave %d / B]", idx);
        print_voltage(label_b, read_copley_voltage(master, idx, copley::Axis::B));
    }

    master.close();
    return 0;
}
