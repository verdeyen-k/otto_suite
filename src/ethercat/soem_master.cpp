#include "ethercat/soem_master.hpp"

#include <cstdio>
#include <cstring>
#include <stdexcept>

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

void SoemMaster::configure_pdos() {
    ecx_config_map_group(ctx_, io_map_.data(), 0);
    ecx_configdc(ctx_);
    for (int i = 1; i <= slave_count_; ++i) {
        if (ctx_->slavelist[i].CoEdetails > 0) {
            ecx_slavembxcyclic(ctx_, i);
        }
    }
}

bool SoemMaster::wait_for_safe_op(int timeout_us) {
    return ecx_statecheck(ctx_, 0, EC_STATE_SAFE_OP, timeout_us) == EC_STATE_SAFE_OP;
}

bool SoemMaster::request_operational_state(int retries, int timeout_us) {
    ctx_->slavelist[0].state = EC_STATE_OPERATIONAL;
    ecx_writestate(ctx_, 0);
    for (int i = 0; i < retries; ++i) {
        send_receive();
        if (ecx_statecheck(ctx_, 0, EC_STATE_OPERATIONAL, timeout_us) == EC_STATE_OPERATIONAL) {
            return true;
        }
    }
    return false;
}

int SoemMaster::send_receive() {
    ecx_send_processdata(ctx_);
    return ecx_receive_processdata(ctx_, EC_TIMEOUTRET);
}

void SoemMaster::close() {
    if (ctx_ != nullptr) {
        ecx_close(ctx_);
    }
}

void SoemMaster::write_output_bytes(int slave_index, int offset, const std::uint8_t *data, std::size_t len) {
    std::memcpy(ctx_->slavelist[slave_index].outputs + offset, data, len);
}

void SoemMaster::read_input_bytes(int slave_index, int offset, std::uint8_t *out, std::size_t len) const {
    std::memcpy(out, ctx_->slavelist[slave_index].inputs + offset, len);
}

void SoemMaster::sdo_write(int slave_index, std::uint16_t index, std::uint8_t subindex, const void *data, int size) {
    ecx_SDOwrite(ctx_, static_cast<std::uint16_t>(slave_index), index, subindex, FALSE, size, data, EC_TIMEOUTRXM);
}

void SoemMaster::sdo_read(int slave_index, std::uint16_t index, std::uint8_t subindex, void *out, int size) const {
    int actual_size = size;
    ecx_SDOread(ctx_, static_cast<std::uint16_t>(slave_index), index, subindex, FALSE, &actual_size, out,
                EC_TIMEOUTRXM);
}

void map_pdo(SoemMaster &master, int slave_index, std::uint16_t pdo_index, const std::vector<PdoMapEntry> &entries) {
    std::uint8_t zero_count = 0;
    master.sdo_write(slave_index, pdo_index, 0, &zero_count, sizeof(zero_count));
    for (std::size_t i = 0; i < entries.size(); ++i) {
        const auto &entry = entries[i];
        std::uint32_t packed = (static_cast<std::uint32_t>(entry.index) << 16) |
                                (static_cast<std::uint32_t>(entry.subindex) << 8) |
                                static_cast<std::uint32_t>(entry.bit_length);
        auto subindex = static_cast<std::uint8_t>(i + 1);
        master.sdo_write(slave_index, pdo_index, subindex, &packed, sizeof(packed));
    }
    auto count = static_cast<std::uint8_t>(entries.size());
    master.sdo_write(slave_index, pdo_index, 0, &count, sizeof(count));
}

void assign_pdos(SoemMaster &master, int slave_index, std::uint16_t sm_assignment_index,
                  const std::vector<std::uint16_t> &pdo_indices) {
    std::uint8_t zero_count = 0;
    master.sdo_write(slave_index, sm_assignment_index, 0, &zero_count, sizeof(zero_count));
    for (std::size_t i = 0; i < pdo_indices.size(); ++i) {
        auto subindex = static_cast<std::uint8_t>(i + 1);
        master.sdo_write(slave_index, sm_assignment_index, subindex, &pdo_indices[i], sizeof(pdo_indices[i]));
    }
    auto count = static_cast<std::uint8_t>(pdo_indices.size());
    master.sdo_write(slave_index, sm_assignment_index, 0, &count, sizeof(count));
}

void assign_single_pdo(SoemMaster &master, int slave_index, std::uint16_t sm_assignment_index,
                        std::uint16_t pdo_index) {
    assign_pdos(master, slave_index, sm_assignment_index, {pdo_index});
}

}  // namespace ethercat
