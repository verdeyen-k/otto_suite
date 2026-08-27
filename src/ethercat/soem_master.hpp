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

    struct SlaveState {
        int slave_index;
        std::string name;
        std::uint16_t al_state;         // raw AL state; bit 0x10 set means SAFE_OP/etc + ERROR
        std::uint16_t al_status_code;   // 0 if none
        std::string al_status_string;   // ec_ALstatuscode2string(al_status_code)
    };

    // Reads and returns EVERY slave's current EtherCAT AL state and status
    // code (ecx_readstate() + per-slave state/ALstatuscode) -- independent
    // of any CiA-402/application-layer interpretation. Useful specifically
    // when request_operational_state()/wait_for_safe_op() fails: that
    // failure is a WHOLE-BUS outcome (state is written to slave 0,
    // broadcast), so a stuck slave completely unrelated to whichever axis
    // a tool happens to be targeting can be the actual reason. Mirrors the
    // diagnostic SOEM's own ec_sample.c/slaveinfo print in their own
    // failure branches.
    [[nodiscard]] std::vector<SlaveState> read_all_slave_states() const;

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

    // Polls (via direct AL status reads, not cyclic PDO exchange -- no
    // send_receive() needed first) until the bus reaches at least SAFE_OP,
    // or times out. SAFE_OP is the minimum needed for input PDO data
    // (statusword, position, etc.) to be valid -- it specifically means
    // "inputs valid, outputs safe/inactive" -- unlike OPERATIONAL, it
    // requires no cyclic process-data exchange to reach, since config_map()
    // already drives the PRE-OP -> SAFE-OP transition. Passive/diagnostic
    // tools that never need outputs applied (e.g. zeroerr_state) should
    // check this instead of request_operational_state(), so a bus that's
    // stuck below OP for some other reason (a fault, a watchdog issue)
    // still yields valid, explainable state instead of a bare abort.
    bool wait_for_safe_op(int timeout_us = 6'000'000);

    // Requests OPERATIONAL state and drives the transition through --
    // reaching OP requires several cycles of valid process-data exchange
    // while the request is pending, not just a single state write. Returns
    // true once OP is confirmed; callers must not assume actuators are
    // safely operable if this returns false.
    //
    // First spends `dc_settle_us` exchanging plain cyclic process data
    // (no state change requested yet) before ever asking for OP -- a
    // distributed-clocks bus (configure_pdos() calls ecx_configdc(); a
    // multi-slave EtherCAT network commonly has DC enabled bus-wide even
    // if this driver's own slaves don't care about sync) needs several
    // cycles of real traffic for slave clocks to converge before they'll
    // accept the OP transition. This mirrors SOEM's own ec_sample.c,
    // which sleeps a full second here for exactly this reason -- skipping
    // it is a real, confirmed-on-hardware way to get stuck at SAFE_OP
    // indefinitely despite every slave being otherwise healthy.
    //
    // The retry loop deliberately keeps sending a process-data cycle
    // every ~1ms throughout (statecheck is sampled with a near-zero
    // timeout, not handed a real one) -- ecx_statecheck's own internal
    // wait only re-reads the AL status register, it does NOT send
    // process data, so giving it a real timeout here reintroduces a gap
    // between cyclic frames. Confirmed on real hardware this gap alone
    // (previously up to 50ms per retry) can trip a slave's sync manager
    // watchdog -- AL state SAFE_OP+ERROR, status "Sync manager
    // watchdog" -- on a different slave each run, which looked like
    // random flakiness until traced to this. retries=3000 at ~1ms each
    // gives a generous ~3s worst-case budget; success is typically near-
    // instant once frames stop having gaps. If this still fails,
    // read_all_slave_states() on failure is what tells you why.
    bool request_operational_state(int retries = 3000, int dc_settle_us = 1'000'000);

    // Exchanges one cycle of process data. Returns the working counter;
    // callers should compare against expected_wkc() to detect a
    // dropped/incomplete cycle.
    int send_receive();

    // The working counter send_receive() SHOULD reach once every mapped
    // slave successfully processes the frame (outputsWKC*2 + inputsWKC,
    // the same formula SOEM's own samples print as "Calculated
    // workcounter"). Valid only after configure_pdos(). A slave that sits
    // at SAFE_OP indefinitely with no AL error at all (see
    // read_all_slave_states()) is worth checking against this -- if
    // send_receive()'s actual return is consistently below this, some
    // slave isn't fully acknowledging the exchange even though nothing
    // is reporting a fault.
    [[nodiscard]] int expected_wkc() const;

    void close();

    // Raw PDO IO access. 1-based slave_index.
    void write_output_bytes(int slave_index, int offset, const std::uint8_t *data, std::size_t len);
    void read_input_bytes(int slave_index, int offset, std::uint8_t *out, std::size_t len) const;

    // Raw SDO/mailbox access. 1-based slave_index. CoE mailbox
    // communication works from PRE-OP onward -- no PDO mapping or OP state
    // required. Both return the SOEM working counter: >0 means the slave
    // actually acknowledged the request; <=0 means it didn't (SDO abort --
    // e.g. the object doesn't exist/isn't supported -- or a mailbox
    // timeout). On a failed read, `out` is left UNTOUCHED (not zeroed) --
    // SOEM's ecx_SDOread never writes the caller's buffer on an abort, so
    // callers that pre-zero their buffer and skip checking this return
    // value cannot tell "genuinely read zero" apart from "read failed."
    // This matters concretely: see CopleyAxis::read_safety_circuit_status.
    int sdo_write(int slave_index, std::uint16_t index, std::uint8_t subindex, const void *data, int size);
    int sdo_read(int slave_index, std::uint16_t index, std::uint8_t subindex, void *out, int size) const;

private:
    static int config_func_trampoline(ecx_context *context, std::uint16_t slave);

    std::string ifname_;
    ecx_context *ctx_ = nullptr;
    bool closed_ = false;
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
// 0), write each PDO index at increasing sub-indices starting at 1,
// re-enable (subindex 0 := count) -- assigns one or more PDOs to the given
// sync-manager assignment object (0x1C12 for RxPDO/SM2, 0x1C13 for
// TxPDO/SM3). More than one PDO in the list is a normal, documented
// CANopen capability (e.g. Copley's manual gives 0x1C12/0x1C13 a
// documented range of 0-4 mapped PDOs) -- used to combine a fixed PDO with
// a custom one, e.g. addressing a second axis on a multi-axis drive.
void assign_pdos(SoemMaster &master, int slave_index, std::uint16_t sm_assignment_index,
                  const std::vector<std::uint16_t> &pdo_indices);

// Convenience for the single-PDO case.
void assign_single_pdo(SoemMaster &master, int slave_index, std::uint16_t sm_assignment_index,
                        std::uint16_t pdo_index);

}  // namespace ethercat
