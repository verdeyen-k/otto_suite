#include "bridge/chassis_link.hpp"

#include <zmq.h>

#include <cstddef>
#include <cstdio>
#include <stdexcept>
#include <string>

namespace bridge {

namespace {
std::string tcp_bind_endpoint(int port) { return "tcp://*:" + std::to_string(port); }
}  // namespace

ChassisLink::ChassisLink(int command_port, int telemetry_port) {
    context_ = zmq_ctx_new();
    if (context_ == nullptr) {
        throw std::runtime_error("zmq_ctx_new failed");
    }

    command_sub_ = zmq_socket(context_, ZMQ_SUB);
    if (command_sub_ == nullptr) {
        throw std::runtime_error("zmq_socket(SUB) failed");
    }
    const int conflate = 1;
    zmq_setsockopt(command_sub_, ZMQ_CONFLATE, &conflate, sizeof(conflate));
    zmq_setsockopt(command_sub_, ZMQ_SUBSCRIBE, "", 0);
    if (zmq_bind(command_sub_, tcp_bind_endpoint(command_port).c_str()) != 0) {
        throw std::runtime_error("zmq_bind(command, port " + std::to_string(command_port) +
                                  ") failed: " + zmq_strerror(zmq_errno()));
    }

    telemetry_pub_ = zmq_socket(context_, ZMQ_PUB);
    if (telemetry_pub_ == nullptr) {
        throw std::runtime_error("zmq_socket(PUB) failed");
    }
    if (zmq_bind(telemetry_pub_, tcp_bind_endpoint(telemetry_port).c_str()) != 0) {
        throw std::runtime_error("zmq_bind(telemetry, port " + std::to_string(telemetry_port) +
                                  ") failed: " + zmq_strerror(zmq_errno()));
    }
}

ChassisLink::~ChassisLink() {
    if (command_sub_ != nullptr) zmq_close(command_sub_);
    if (telemetry_pub_ != nullptr) zmq_close(telemetry_pub_);
    if (context_ != nullptr) zmq_ctx_term(context_);
}

std::optional<kinematics::ChassisSpeeds> ChassisLink::try_receive_command() {
    ChassisCommandWire wire{};
    // ZMQ_CONFLATE means at most one message is ever queued; ZMQ_DONTWAIT
    // returns -1/EAGAIN immediately if nothing new has arrived since the
    // last successful recv, rather than blocking the real-time cycle.
    int rc = zmq_recv(command_sub_, &wire, sizeof(wire), ZMQ_DONTWAIT);
    if (rc < 0) {
        return std::nullopt;
    }
    if (rc != static_cast<int>(sizeof(wire))) {
        std::fprintf(stderr, "bridge: command message wrong size (%d, expected %zu) -- ignoring\n", rc,
                     sizeof(wire));
        return std::nullopt;
    }
    return kinematics::ChassisSpeeds{wire.vx_mps, wire.vy_mps, wire.omega_rad_per_s};
}

void ChassisLink::publish_telemetry(const kinematics::ChassisSpeeds &speeds, bool any_fault,
                                     const std::array<ModuleTelemetry, 4> &modules) {
    ChassisTelemetryWire wire{};
    wire.vx_mps = speeds.vx_mps;
    wire.vy_mps = speeds.vy_mps;
    wire.omega_rad_per_s = speeds.omega_rad_per_s;
    wire.any_fault = any_fault ? 1 : 0;
    for (std::size_t i = 0; i < modules.size(); ++i) {
        wire.modules[i].steer_angle_deg = modules[i].angle_deg;
        wire.modules[i].drive_speed_mps = modules[i].speed_mps;
        wire.modules[i].has_fault = modules[i].has_fault ? 1 : 0;
    }
    zmq_send(telemetry_pub_, &wire, sizeof(wire), ZMQ_DONTWAIT);
}

}  // namespace bridge
