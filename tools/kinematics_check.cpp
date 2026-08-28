// Pure-math sanity check for the swerve kinematics module -- no EtherCAT
// bus, no hardware, safe to run anywhere. Confirms known chassis-motion
// cases produce the expected module states, that the forward/inverse pair
// round-trips, and that optimize() actually shortens the steering delta.
// Exits non-zero on the first failed check.
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <exception>
#include <string>

#include "kinematics/swerve_kinematics.hpp"
#include "robot/robot_config.hpp"

namespace {

constexpr double kEpsilon = 1e-6;
int g_failures = 0;

void expect_near(double actual, double expected, const char *what) {
    if (std::abs(actual - expected) > kEpsilon) {
        std::fprintf(stderr, "FAIL %s: expected %.6f, got %.6f\n", what, expected, actual);
        ++g_failures;
    }
}

}  // namespace

int main(int argc, char **argv) {
    const std::string config_path = argc > 1 ? argv[1] : robot::kDefaultConfigPath;
    robot::RobotConfig cfg;
    try {
        cfg = robot::load_robot_config(config_path);
    } catch (const std::exception &e) {
        std::fprintf(stderr, "error: %s\n", e.what());
        return 1;
    }
    const auto module_positions = cfg.module_positions();

    kinematics::SwerveKinematics k(
        {module_positions[0], module_positions[1], module_positions[2], module_positions[3]});

    // Straight forward: every module points at angle 0, speed 1.
    {
        auto states = k.to_module_states({1.0, 0.0, 0.0});
        for (const auto &s : states) {
            expect_near(s.speed_mps, 1.0, "forward speed");
            expect_near(s.angle_rad, 0.0, "forward angle");
        }
    }

    // Pure strafe left: every module points at +90 deg, speed 1.
    {
        auto states = k.to_module_states({0.0, 1.0, 0.0});
        for (const auto &s : states) {
            expect_near(s.speed_mps, 1.0, "strafe speed");
            expect_near(s.angle_rad, M_PI / 2.0, "strafe angle");
        }
    }

    // Pure rotation in place: every module's speed is equal (symmetric
    // chassis), and each module points perpendicular to its own radius arm.
    {
        auto states = k.to_module_states({0.0, 0.0, 1.0});
        const double expected_speed =
            std::hypot(cfg.wheelbase_length_m / 2.0, cfg.wheelbase_width_m / 2.0);
        for (std::size_t i = 0; i < states.size(); ++i) {
            expect_near(states[i].speed_mps, expected_speed, "rotate-in-place speed");
            const double expected_angle =
                std::atan2(module_positions[i].x_m, -module_positions[i].y_m);
            expect_near(states[i].angle_rad, expected_angle, "rotate-in-place angle");
        }
    }

    // Round-trip: an arbitrary combined motion, through to_module_states
    // then back through to_chassis_speeds, should reproduce the original.
    {
        const kinematics::ChassisSpeeds original{0.7, -0.3, 0.5};
        auto states = k.to_module_states(original);
        auto recovered = k.to_chassis_speeds(states);
        expect_near(recovered.vx_mps, original.vx_mps, "round-trip vx");
        expect_near(recovered.vy_mps, original.vy_mps, "round-trip vy");
        expect_near(recovered.omega_rad_per_s, original.omega_rad_per_s, "round-trip omega");
    }

    // optimize(): a target 170 degrees away from current should flip to a
    // 10-degree move with reversed speed instead.
    {
        const double current_angle = 0.0;
        const kinematics::ModuleState desired{1.0, 170.0 * M_PI / 180.0};
        auto opt = kinematics::SwerveKinematics::optimize(desired, current_angle);
        expect_near(opt.speed_mps, -1.0, "optimize flipped speed");
        expect_near(opt.angle_rad, -10.0 * M_PI / 180.0, "optimize flipped angle");
    }

    // optimize(): a target 30 degrees away should pass through unchanged.
    {
        const double current_angle = 0.0;
        const kinematics::ModuleState desired{1.0, 30.0 * M_PI / 180.0};
        auto opt = kinematics::SwerveKinematics::optimize(desired, current_angle);
        expect_near(opt.speed_mps, 1.0, "optimize unflipped speed");
        expect_near(opt.angle_rad, 30.0 * M_PI / 180.0, "optimize unflipped angle");
    }

    // desaturate_wheel_speeds(): a combined translate+rotate command whose
    // fastest module exceeds the cap should scale every module's speed
    // down by the same factor, changing no module's angle.
    {
        auto states = k.to_module_states({0.6, 0.0, 2.0});
        double max_before = 0.0;
        for (const auto &s : states) max_before = std::max(max_before, std::abs(s.speed_mps));
        const auto before = states;
        const double cap = max_before / 2.0;  // deliberately under the fastest module
        kinematics::SwerveKinematics::desaturate_wheel_speeds(states, cap);
        double max_after = 0.0;
        for (std::size_t i = 0; i < states.size(); ++i) {
            expect_near(states[i].angle_rad, before[i].angle_rad, "desaturate preserves angle");
            expect_near(states[i].speed_mps / before[i].speed_mps, 0.5, "desaturate uniform scale");
            max_after = std::max(max_after, std::abs(states[i].speed_mps));
        }
        expect_near(max_after, cap, "desaturate caps fastest module");
    }

    // Bus locations: 4 distinct steer slaves, and 4 distinct (drive slave,
    // axis) pairs -- catches a copy-paste config mistake (e.g. two modules
    // pointed at the same steer actuator) without needing hardware.
    {
        for (std::size_t i = 0; i < cfg.module_bus_locations.size(); ++i) {
            for (std::size_t j = i + 1; j < cfg.module_bus_locations.size(); ++j) {
                const auto &a = cfg.module_bus_locations[i];
                const auto &b = cfg.module_bus_locations[j];
                if (a.steer_slave == b.steer_slave) {
                    std::fprintf(stderr, "FAIL bus locations: module %zu and %zu share steer slave %d\n", i, j,
                                 a.steer_slave);
                    ++g_failures;
                }
                if (a.drive_slave == b.drive_slave && a.drive_axis == b.drive_axis) {
                    std::fprintf(stderr, "FAIL bus locations: module %zu and %zu share drive slave %d axis %c\n",
                                 i, j, a.drive_slave, a.drive_axis);
                    ++g_failures;
                }
            }
        }
    }

    // desaturate_wheel_speeds(): under the cap already -- no-op.
    {
        auto states = k.to_module_states({0.1, 0.0, 0.0});
        const auto before = states;
        kinematics::SwerveKinematics::desaturate_wheel_speeds(states, 10.0);
        for (std::size_t i = 0; i < states.size(); ++i) {
            expect_near(states[i].speed_mps, before[i].speed_mps, "desaturate no-op speed");
            expect_near(states[i].angle_rad, before[i].angle_rad, "desaturate no-op angle");
        }
    }

    if (g_failures == 0) {
        std::printf("All kinematics checks passed.\n");
        return 0;
    }
    std::fprintf(stderr, "%d kinematics check(s) failed.\n", g_failures);
    return 1;
}
