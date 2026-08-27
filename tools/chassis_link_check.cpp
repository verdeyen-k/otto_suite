// Hardware-free round-trip smoke test for the ZeroMQ link (src/bridge/) --
// no EtherCAT, no Python, safe to run anywhere libzmq is installed.
// Confirms the actual wire encoding/decoding and socket behavior: a raw
// PUB "pretending to be teleop_joystick.py" sends a command, ChassisLink
// (bound as the real-time core would be) decodes it correctly; a raw SUB
// "pretending to be teleop_joystick.py" receives what ChassisLink
// publishes as telemetry; and a second try_receive_command() call with
// nothing new correctly reports std::nullopt (proves ZMQ_CONFLATE +
// non-blocking recv work as intended, not just "recv happened to work").
//
// PUB/SUB has a well-known "slow joiner" issue -- a subscriber that just
// connected may not be registered with the publisher yet -- so both
// directions retry for a few seconds rather than sending/checking once.
#include <zmq.h>

#include <chrono>
#include <cmath>
#include <cstdio>
#include <optional>
#include <string>
#include <thread>

#include "kinematics/swerve_kinematics.hpp"
#include "bridge/chassis_link.hpp"
#include "bridge/messages.hpp"

namespace {

constexpr int kTestCommandPort = 15555;
constexpr int kTestTelemetryPort = 15556;
int g_failures = 0;

void expect(bool condition, const char *what) {
    if (!condition) {
        std::fprintf(stderr, "FAIL: %s\n", what);
        ++g_failures;
    }
}

void expect_near(double actual, double expected, const char *what) {
    if (std::abs(actual - expected) > 1e-9) {
        std::fprintf(stderr, "FAIL %s: expected %.6f, got %.6f\n", what, expected, actual);
        ++g_failures;
    }
}

}  // namespace

int main() {
    bridge::ChassisLink chassis_link(kTestCommandPort, kTestTelemetryPort);

    void *ctx = zmq_ctx_new();

    // Raw PUB standing in for teleop_joystick.py, connecting to the
    // command SUB that ChassisLink just bound.
    void *fake_teleop_pub = zmq_socket(ctx, ZMQ_PUB);
    zmq_connect(fake_teleop_pub, ("tcp://127.0.0.1:" + std::to_string(kTestCommandPort)).c_str());

    // Raw SUB standing in for teleop_joystick.py, connecting to the
    // telemetry PUB that ChassisLink just bound.
    void *fake_teleop_sub = zmq_socket(ctx, ZMQ_SUB);
    zmq_setsockopt(fake_teleop_sub, ZMQ_SUBSCRIBE, "", 0);
    zmq_connect(fake_teleop_sub, ("tcp://127.0.0.1:" + std::to_string(kTestTelemetryPort)).c_str());

    // Command direction: retry sending until ChassisLink actually decodes
    // it, working around the slow-joiner window.
    bridge::ChassisCommandWire command_wire{1.5, -2.5, 0.75};
    std::optional<kinematics::ChassisSpeeds> received;
    for (int attempt = 0; attempt < 40 && !received.has_value(); ++attempt) {
        zmq_send(fake_teleop_pub, &command_wire, sizeof(command_wire), 0);
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        received = chassis_link.try_receive_command();
    }
    expect(received.has_value(), "command was received at all");
    if (received.has_value()) {
        expect_near(received->vx_mps, 1.5, "command vx");
        expect_near(received->vy_mps, -2.5, "command vy");
        expect_near(received->omega_rad_per_s, 0.75, "command omega");
    }

    // Conflate + non-blocking: with nothing new sent, a second call must
    // report nullopt, not the same message replayed.
    auto second_call = chassis_link.try_receive_command();
    expect(!second_call.has_value(), "no new command -> nullopt, not a replay");

    // Telemetry direction: retry receiving until the slow-joiner window
    // closes.
    kinematics::ChassisSpeeds telemetry_speeds{0.3, 0.2, -0.1};
    bridge::ChassisTelemetryWire telemetry_wire{};
    bool telemetry_received = false;
    for (int attempt = 0; attempt < 40 && !telemetry_received; ++attempt) {
        chassis_link.publish_telemetry(telemetry_speeds, /*any_fault=*/true);
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        int rc = zmq_recv(fake_teleop_sub, &telemetry_wire, sizeof(telemetry_wire), ZMQ_DONTWAIT);
        if (rc == static_cast<int>(sizeof(telemetry_wire))) {
            telemetry_received = true;
        }
    }
    expect(telemetry_received, "telemetry was received at all");
    if (telemetry_received) {
        expect_near(telemetry_wire.vx_mps, 0.3, "telemetry vx");
        expect_near(telemetry_wire.vy_mps, 0.2, "telemetry vy");
        expect_near(telemetry_wire.omega_rad_per_s, -0.1, "telemetry omega");
        expect(telemetry_wire.any_fault == 1, "telemetry any_fault flag");
    }

    zmq_close(fake_teleop_pub);
    zmq_close(fake_teleop_sub);
    zmq_ctx_term(ctx);

    if (g_failures == 0) {
        std::printf("All chassis_link checks passed.\n");
        return 0;
    }
    std::fprintf(stderr, "%d chassis_link check(s) failed.\n", g_failures);
    return 1;
}
