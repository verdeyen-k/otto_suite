#include "ethercat/soem_master.hpp"

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <stdexcept>
#include <thread>

#include "soem/soem.h"

namespace ethercat {

namespace {
constexpr std::size_t kIoMapBytes = 4096;
}  // namespace

SoemMaster::SoemMaster(std::string ifname) : ifname_(std::move(ifname)), io_map_(kIoMapBytes, 0) {
    ctx_ = new ecx_context{};
}

SoemMaster::~SoemMaster() {
    close();
    delete ctx_;
}

int SoemMaster::scan() {
    if (ecx_init(ctx_, ifname_.c_str()) <= 0) {
        throw std::runtime_error("SoemMaster::scan: ecx_init failed to open interface '" + ifname_ +
                                  "' (not found, or needs root/CAP_NET_RAW)");
    }
    slave_count_ = ecx_config_init(ctx_);
    if (slave_count_ <= 0) {
        throw std::runtime_error("SoemMaster::scan: no EtherCAT slaves found on interface '" + ifname_ + "'");
    }
    ctx_->userdata = this;
    return slave_count_;
}

std::string SoemMaster::slave_name(int slave_index) const { return ctx_->slavelist[slave_index].name; }

std::uint32_t SoemMaster::slave_vendor_id(int slave_index) const { return ctx_->slavelist[slave_index].eep_man; }

std::uint32_t SoemMaster::slave_product_code(int slave_index) const { return ctx_->slavelist[slave_index].eep_id; }

std::vector<SoemMaster::SlaveState> SoemMaster::read_all_slave_states() const {
    ecx_readstate(ctx_);
    std::vector<SlaveState> states;
    states.reserve(slave_count_);
    for (int i = 1; i <= slave_count_; ++i) {
        const auto &slave = ctx_->slavelist[i];
        states.push_back(SlaveState{i, slave.name, slave.state, slave.ALstatuscode,
                                     ec_ALstatuscode2string(slave.ALstatuscode)});
    }
    return states;
}

std::vector<int> SoemMaster::find_slaves_by_identity(std::uint32_t vendor_id, std::uint32_t product_code) const {
    std::vector<int> matches;
    for (int i = 1; i <= slave_count_; ++i) {
        if (ctx_->slavelist[i].eep_man == vendor_id && ctx_->slavelist[i].eep_id == product_code) {
            matches.push_back(i);
        }
    }
    return matches;
}

int SoemMaster::config_func_trampoline(ecx_context *context, std::uint16_t slave) {
    // SOEM invokes this with a 1-based slave position (see PO2SOconfig's
    // doc comment in ec_main.h) -- printed explicitly so a mismatch against
    // our own 1-based bookkeeping is visible immediately rather than
    // silently misconfiguring a neighboring slave.
    std::fprintf(stderr, "soem_master: config_func invoked for slave position %u (1-based)\n", slave);
    auto *self = static_cast<SoemMaster *>(context->userdata);
    auto it = self->config_funcs_.find(static_cast<int>(slave));
    if (it != self->config_funcs_.end()) {
        it->second(*self, static_cast<int>(slave));
    }
    return 1;
}

void SoemMaster::set_config_func(int slave_index, std::function<void(SoemMaster &, int)> fn) {
    config_funcs_[slave_index] = std::move(fn);
    ctx_->slavelist[slave_index].PO2SOconfig = &SoemMaster::config_func_trampoline;
}

void SoemMaster::configure_pdos(std::uint32_t dc_sync0_cycle_ns) {
    // The bring-up loops below pace themselves at this same rate -- see
    // their comments on why warming up at a different cadence than the
    // SYNC0 period the slaves are about to be told to expect is itself a
    // real inconsistency, independent of jitter/precision.
    cycle_period_ = std::chrono::nanoseconds(dc_sync0_cycle_ns);
    ecx_config_map_group(ctx_, io_map_.data(), 0);
    ecx_configdc(ctx_);
    // ecx_configdc() alone does not start SYNC0 pulse generation -- see
    // the doc comment in the header. Explicitly start it for every
    // DC-capable slave.
    for (int i = 1; i <= slave_count_; ++i) {
        if (ctx_->slavelist[i].hasdc) {
            ecx_dcsync0(ctx_, static_cast<std::uint16_t>(i), TRUE, dc_sync0_cycle_ns, 0);
        }
    }
    // Deliberately NOT calling ecx_slavembxcyclic() here (SOEM's own
    // ec_sample.c does, for every CoE slave). That switches a slave's
    // mailbox into a queued/cyclic mode which requires periodic
    // ecx_mbxhandler() calls to actually service the queue -- we never
    // call ecx_mbxhandler anywhere (we don't need a real-time mailbox
    // pump, just occasional blocking SDO reads/writes), so every SDO
    // transaction after ecx_slavembxcyclic() would just queue, time out
    // waiting to be serviced, and fail. Confirmed on real hardware: every
    // SDO read succeeded during config_func (which runs before this
    // point) and every one failed afterward, on a completely healthy
    // slave -- this was the cause, not anything wrong with the slave.
}

bool SoemMaster::wait_for_safe_op(int timeout_us) {
    // Same fix already applied to request_operational_state(), never
    // carried over here: a real timeout handed to ecx_statecheck makes it
    // spend that whole wait internally polling AL status via BRD only --
    // it never sends real cyclic process data. That means, until this
    // function returns, a DC-capable slave's clock (and now its SYNC0
    // pulse, started in configure_pdos()) has had zero real cyclic frames
    // to lock onto, no matter how long request_operational_state()'s own
    // settle period runs afterward. Pump process data ourselves instead,
    // sampling state with a near-zero timeout, so real cyclic exchange
    // (and DC convergence) starts as early as possible -- right after
    // configure_pdos(), not only once SAFE_OP is already confirmed.
    //
    // sleep_until an absolute, precomputed wake time rather than
    // sleep_for(cycle_period_) after each iteration's work: a relative
    // sleep means the actual cycle period is cycle_period_ *plus*
    // however long send_receive()/ecx_statecheck() took that iteration,
    // which varies and compounds over the loop -- a real timing-precision
    // bug in this loop itself, independent of and not fixed by OS
    // scheduling priority/affinity. Paced at cycle_period_ (set by
    // configure_pdos()), not a hardcoded step, so bring-up runs at the
    // same rate the slaves were actually told to expect via SYNC0.
    const auto cycle_us = std::chrono::duration_cast<std::chrono::microseconds>(cycle_period_).count();
    const int retries =
        static_cast<int>(std::max<std::int64_t>(1, timeout_us / std::max<std::int64_t>(1, cycle_us)));
    auto next_wake = std::chrono::steady_clock::now();
    for (int i = 0; i < retries; ++i) {
        send_receive();
        if (ecx_statecheck(ctx_, 0, EC_STATE_SAFE_OP, 0) == EC_STATE_SAFE_OP) {
            return true;
        }
        next_wake += cycle_period_;
        std::this_thread::sleep_until(next_wake);
    }
    return false;
}

bool SoemMaster::request_operational_state(int retries, int dc_settle_us) {
    if (dc_settle_us > 0) {
        // Absolute-time scheduling here too -- see wait_for_safe_op()'s
        // comment on why a relative sleep_for after variable-length work
        // is itself a source of cycle-to-cycle jitter. Paced at
        // cycle_period_, not a hardcoded step, for the same reason.
        auto next_wake = std::chrono::steady_clock::now();
        auto settle_until = next_wake + std::chrono::microseconds(dc_settle_us);
        while (next_wake < settle_until) {
            send_receive();
            next_wake += cycle_period_;
            std::this_thread::sleep_until(next_wake);
        }
    }
    // `timeout_us` is intentionally NOT handed to ecx_statecheck here:
    // that function polls the AL status register in its own internal
    // loop (a separate BRD read, not process data) and does not send any
    // cyclic frames while it waits -- passing it a real timeout here
    // reintroduces exactly the gap that trips a slave's sync manager
    // watchdog (confirmed on real hardware: AL state SAFE_OP+ERROR,
    // status "Sync manager watchdog", on a different slave each run).
    // Instead we keep control of the pacing ourselves: one process-data
    // cycle, one near-instant state sample, repeat -- so cyclic frames
    // never stop flowing throughout the whole retry window.
    //
    // ecx_writestate() is its own datagram, separate from the cyclic
    // process-data frames send_receive() exchanges every cycle -- if that
    // one broadcast request is dropped or corrupted for a given slave (more
    // plausible on a bigger topology with more hops through multiple
    // junctions), that slave never learns it should transition and will
    // sit at SAFE_OP indefinitely no matter how long the loop below keeps
    // sampling, since nothing re-sends the request. Confirmed on real
    // hardware: on a 9-slave bus, a different, seemingly healthy subset of
    // slaves (clean working counter, no AL error) fails to reach
    // OPERATIONAL each run -- true non-determinism, not a per-slave
    // config/hardware issue, exactly what a dropped one-shot state-request
    // frame would produce. Re-issuing it periodically (not just once)
    // gives a dropped request another chance.
    // ~50ms between resends, expressed in cycles of whatever
    // cycle_period_ actually is rather than an assumed 1ms step.
    const int kResendEveryNRetries =
        std::max(1, static_cast<int>(std::chrono::milliseconds(50) / cycle_period_));
    // Absolute-time scheduling here too -- see the comment in
    // wait_for_safe_op() on why a relative sleep after variable-length
    // work is itself a source of cycle-to-cycle jitter, independent of OS
    // scheduling. Paced at cycle_period_, not a hardcoded step.
    auto next_wake = std::chrono::steady_clock::now();
    for (int i = 0; i < retries; ++i) {
        if (i % kResendEveryNRetries == 0) {
            // Per not-yet-OPERATIONAL slave (individually, an FPWR, not
            // just the slave-0 broadcast below), send two things every
            // resend cycle, mirroring SOEM's own recovery logic in
            // ec_sample.c's ecatcheck() thread:
            //   1. An ACK (state | EC_STATE_ACK -- same bit value as the
            //      AL status ERROR bit, e.g. SAFE_OP+ERROR = 0x14 "sync
            //      manager watchdog"). Not self-clearing -- a slave latched
            //      in it stays stuck across any number of process restarts
            //      until explicitly ACKed (confirmed on real hardware: only
            //      a full reboot, forcing the link down/up and resetting
            //      the slave's own ESC, ever cleared it otherwise).
            //   2. A direct OPERATIONAL request to that specific slave.
            //      The more common symptom seen is a slave sitting at
            //      plain SAFE_OP with no error bit at all, which the ACK
            //      alone does nothing for -- and confirmed on real
            //      hardware that dozens of broadcast-only OPERATIONAL
            //      requests over 3 seconds did not unstick one either. An
            //      individually addressed write is the one thing in the
            //      reference recovery path we hadn't tried yet.
            // Both writes are harmless no-ops for a slave that isn't
            // actually in either situation.
            ecx_readstate(ctx_);
            for (int slave = 1; slave <= slave_count_; ++slave) {
                if (ctx_->slavelist[slave].state != EC_STATE_OPERATIONAL) {
                    std::fprintf(stderr, "soem_master: slave %d not yet OPERATIONAL (state=0x%02X, %s) -- "
                                          "acknowledging\n",
                                 slave, ctx_->slavelist[slave].state,
                                 ec_ALstatuscode2string(ctx_->slavelist[slave].ALstatuscode));
                    ctx_->slavelist[slave].state = EC_STATE_SAFE_OP | EC_STATE_ACK;
                    ecx_writestate(ctx_, slave);
                    ctx_->slavelist[slave].state = EC_STATE_OPERATIONAL;
                    ecx_writestate(ctx_, slave);
                }
            }
            ctx_->slavelist[0].state = EC_STATE_OPERATIONAL;
            ecx_writestate(ctx_, 0);
        }
        send_receive();
        if (ecx_statecheck(ctx_, 0, EC_STATE_OPERATIONAL, 0) == EC_STATE_OPERATIONAL) {
            return true;
        }
        next_wake += cycle_period_;
        std::this_thread::sleep_until(next_wake);
    }
    return false;
}

int SoemMaster::send_receive() {
    ecx_send_processdata(ctx_);
    return ecx_receive_processdata(ctx_, EC_TIMEOUTRET);
}

int SoemMaster::expected_wkc() const {
    const auto &group = ctx_->grouplist[0];
    return group.outputsWKC * 2 + group.inputsWKC;
}

void SoemMaster::close() {
    // Idempotent: the destructor calls this unconditionally, so a caller
    // that also calls it explicitly (as every tool's normal-exit path
    // does, right before main() returns and the stack-allocated master
    // goes out of scope) would otherwise call ecx_close() on the same
    // context twice. Confirmed on real hardware: that second call is a
    // double free ("free(): double free detected in tcache 2"), crashing
    // right after a fully successful run -- every early-return/error path
    // happened to only close once, which is why this went unnoticed until
    // the first run that ran to completion normally.
    if (ctx_ != nullptr && !closed_) {
        ecx_close(ctx_);
        closed_ = true;
    }
}

void SoemMaster::write_output_bytes(int slave_index, int offset, const std::uint8_t *data, std::size_t len) {
    std::memcpy(ctx_->slavelist[slave_index].outputs + offset, data, len);
}

void SoemMaster::read_input_bytes(int slave_index, int offset, std::uint8_t *out, std::size_t len) const {
    std::memcpy(out, ctx_->slavelist[slave_index].inputs + offset, len);
}

int SoemMaster::sdo_write(int slave_index, std::uint16_t index, std::uint8_t subindex, const void *data, int size) {
    return ecx_SDOwrite(ctx_, static_cast<std::uint16_t>(slave_index), index, subindex, FALSE, size, data,
                         EC_TIMEOUTRXM);
}

int SoemMaster::sdo_read(int slave_index, std::uint16_t index, std::uint8_t subindex, void *out, int size) const {
    int actual_size = size;
    return ecx_SDOread(ctx_, static_cast<std::uint16_t>(slave_index), index, subindex, FALSE, &actual_size, out,
                        EC_TIMEOUTRXM);
}

namespace {
// A silently-failed SDO write during PDO configuration is exactly the
// kind of thing that produces "nothing happens and I can't tell why"
// symptoms later -- print loudly (config_func runs once at connect time,
// not in a hot loop, so this is cheap) rather than letting it pass
// unnoticed like SoemMaster::sdo_write's return value otherwise allows.
void checked_sdo_write(SoemMaster &master, int slave_index, std::uint16_t index, std::uint8_t subindex,
                        const void *data, int size, const char *what) {
    int wkc = master.sdo_write(slave_index, index, subindex, data, size);
    if (wkc <= 0) {
        std::fprintf(stderr,
                     "soem_master: SDO write FAILED (wkc=%d) slave=%d index=0x%04X:%u (%s) -- "
                     "configuration did NOT take effect\n",
                     wkc, slave_index, index, subindex, what);
    }
}
}  // namespace

void map_pdo(SoemMaster &master, int slave_index, std::uint16_t pdo_index, const std::vector<PdoMapEntry> &entries) {
    std::uint8_t zero_count = 0;
    checked_sdo_write(master, slave_index, pdo_index, 0, &zero_count, sizeof(zero_count), "disable PDO for remap");
    for (std::size_t i = 0; i < entries.size(); ++i) {
        const auto &entry = entries[i];
        std::uint32_t packed = (static_cast<std::uint32_t>(entry.index) << 16) |
                                (static_cast<std::uint32_t>(entry.subindex) << 8) |
                                static_cast<std::uint32_t>(entry.bit_length);
        auto subindex = static_cast<std::uint8_t>(i + 1);
        checked_sdo_write(master, slave_index, pdo_index, subindex, &packed, sizeof(packed), "map PDO entry");
    }
    auto count = static_cast<std::uint8_t>(entries.size());
    checked_sdo_write(master, slave_index, pdo_index, 0, &count, sizeof(count), "re-enable PDO with new mapping");
}

void assign_pdos(SoemMaster &master, int slave_index, std::uint16_t sm_assignment_index,
                  const std::vector<std::uint16_t> &pdo_indices) {
    std::uint8_t zero_count = 0;
    checked_sdo_write(master, slave_index, sm_assignment_index, 0, &zero_count, sizeof(zero_count),
                       "disable SM assignment for remap");
    for (std::size_t i = 0; i < pdo_indices.size(); ++i) {
        auto subindex = static_cast<std::uint8_t>(i + 1);
        checked_sdo_write(master, slave_index, sm_assignment_index, subindex, &pdo_indices[i],
                           sizeof(pdo_indices[i]), "assign PDO to SM");
    }
    auto count = static_cast<std::uint8_t>(pdo_indices.size());
    checked_sdo_write(master, slave_index, sm_assignment_index, 0, &count, sizeof(count),
                       "re-enable SM with new assignment");
}

void assign_single_pdo(SoemMaster &master, int slave_index, std::uint16_t sm_assignment_index,
                        std::uint16_t pdo_index) {
    assign_pdos(master, slave_index, sm_assignment_index, {pdo_index});
}

}  // namespace ethercat
