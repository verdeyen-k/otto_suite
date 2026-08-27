// Thin C++ wrapper around SOEM's C API (v2.0.0, the `ecx_`-prefixed,
// explicit-context flavor -- see third_party/SOEM/samples/ec_sample for the
// reference usage this mirrors). Deliberately minimal: just enough surface
// for a single ZeroErr eRob axis today, not a general framework yet.
#pragma once

#include <cstddef>
#include <cstdint>
#include <functional>
#include <string>
#include <unordered_map>
#include <vector>

// Forward-declared rather than including <soem/soem.h> here, so callers of
// this header don't need SOEM's include path. Defined fully in the .cpp.
struct ecx_context;

namespace ethercat {

class SoemMaster {
public:
    explicit SoemMaster(std::string ifname);
    ~SoemMaster();

    SoemMaster(const SoemMaster &) = delete;
    SoemMaster &operator=(const SoemMaster &) = delete;

    // Opens the interface and enumerates slaves (ecx_init + ecx_config_init).
    // Returns the number of slaves found. Throws std::runtime_error on
    // failure (no socket, or zero slaves found).
    int scan();

    [[nodiscard]] int slave_count() const { return slave_count_; }
    [[nodiscard]] std::string slave_name(int slave_index) const;
    [[nodiscard]] std::uint32_t slave_vendor_id(int slave_index) const;
    [[nodiscard]] std::uint32_t slave_product_code(int slave_index) const;

    // Returns 1-based slave positions matching the given identity. Call
    // after scan().
    [[nodiscard]] std::vector<int> find_slaves_by_identity(std::uint32_t vendor_id, std::uint32_t product_code) const;

    // Registers `fn(master, slave_index)` to run during this slave's
    // PRE-OP -> SAFE-OP transition, before PDO mapping is finalized. Must
    // be called after scan(), before configure_pdos(). `slave_index` is
    // 1-based, matching SOEM's own slave numbering -- see the trampoline in
    // the .cpp, which prints the position SOEM actually invokes it with so
    // an indexing mismatch is visible immediately rather than silently
    // misconfiguring a neighboring slave (a real bug hit by an earlier,
    // Python-based version of this project).
    void set_config_func(int slave_index, std::function<void(SoemMaster &, int)> fn);

    // Runs PDO mapping (ecx_config_map_group) + distributed clock config.
    // Triggers every registered config_func.
    void configure_pdos();

    // Requests OPERATIONAL state and drives the transition through --
    // reaching OP requires several cycles of valid process-data exchange
    // while the request is pending, not just a single state write. Returns
    // true once OP is confirmed; callers must not assume actuators are
    // safely operable if this returns false.
    bool request_operational_state(int retries = 40, int timeout_us = 50000);

    // Exchanges one cycle of process data. Returns the working counter;
    // callers should compare against the expected value to detect a
    // dropped/incomplete cycle.
    int send_receive();

    void close();

    // Raw PDO IO access. 1-based slave_index.
    void write_output_bytes(int slave_index, int offset, const std::uint8_t *data, std::size_t len);
    void read_input_bytes(int slave_index, int offset, std::uint8_t *out, std::size_t len) const;

    // Raw SDO/mailbox access. 1-based slave_index. CoE mailbox
    // communication works from PRE-OP onward -- no PDO mapping or OP state
    // required.
    void sdo_write(int slave_index, std::uint16_t index, std::uint8_t subindex, const void *data, int size);
    void sdo_read(int slave_index, std::uint16_t index, std::uint8_t subindex, void *out, int size) const;

private:
    static int config_func_trampoline(ecx_context *context, std::uint16_t slave);

    std::string ifname_;
    ecx_context *ctx_ = nullptr;
    int slave_count_ = 0;
    std::vector<std::uint8_t> io_map_;
    std::unordered_map<int, std::function<void(SoemMaster &, int)>> config_funcs_;
};

// One (object index, subindex, bit length) entry of a PDO's mapping list.
struct PdoMapEntry {
    std::uint16_t index;
    std::uint8_t subindex;
    std::uint8_t bit_length;
};

// The standard CANopen "map a PDO" SDO procedure: disable, write each
// entry's (index, subindex, bit length) as a packed 32-bit value at
// increasing sub-indices, re-enable with the entry count. Only valid on a
// PDO slot the device documents as user-mappable (for the ZeroErr eRob,
// that's 0x1600/0x1A00 specifically -- see the eRob CANopen/EtherCAT User
// Manual's PDO communication section, which explicitly calls out these two
// slots as supporting arbitrary mapping, unlike 0x1601-0x1606/0x1A01-0x1A04).
void map_pdo(SoemMaster &master, int slave_index, std::uint16_t pdo_index, const std::vector<PdoMapEntry> &entries);

// The standard CANopen SM PDO assignment sequence: disable (subindex 0 :=
// 0), point subindex 1 at `pdo_index`, re-enable (subindex 0 := 1) --
// assigns exactly one PDO to the given sync-manager assignment object
// (0x1C12 for RxPDO/SM2, 0x1C13 for TxPDO/SM3).
void assign_single_pdo(SoemMaster &master, int slave_index, std::uint16_t sm_assignment_index,
                        std::uint16_t pdo_index);

}  // namespace ethercat
