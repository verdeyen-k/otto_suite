#include "kinematics/swerve_kinematics.hpp"

#include <cmath>

namespace kinematics {

namespace {

// Solves the 3x3 linear system M x = b via Cramer's rule. Used to solve the
// normal equations (A^T A) x = A^T b for forward kinematics's least-squares
// fit -- a closed-form 3x3 solve is simpler than pulling in a matrix library
// for a problem this small.
std::array<double, 3> solve3x3(const std::array<std::array<double, 3>, 3> &m,
                                const std::array<double, 3> &b) {
    auto det3 = [](const std::array<std::array<double, 3>, 3> &a) {
        return a[0][0] * (a[1][1] * a[2][2] - a[1][2] * a[2][1]) -
               a[0][1] * (a[1][0] * a[2][2] - a[1][2] * a[2][0]) +
               a[0][2] * (a[1][0] * a[2][1] - a[1][1] * a[2][0]);
    };
    const double det = det3(m);
    std::array<double, 3> x{};
    for (int col = 0; col < 3; ++col) {
        std::array<std::array<double, 3>, 3> m_col_replaced = m;
        for (int row = 0; row < 3; ++row) m_col_replaced[row][col] = b[row];
        x[col] = det3(m_col_replaced) / det;
    }
    return x;
}

}  // namespace

SwerveKinematics::SwerveKinematics(std::array<robot::Translation2d, 4> module_positions)
    : module_positions_(module_positions) {}

std::array<ModuleState, 4> SwerveKinematics::to_module_states(const ChassisSpeeds &speeds) const {
    std::array<ModuleState, 4> states{};
    for (std::size_t i = 0; i < module_positions_.size(); ++i) {
        const double x = module_positions_[i].x_m;
        const double y = module_positions_[i].y_m;
        // Velocity at this module = translational velocity + omega x r,
        // where r = (x, y) and omega is about +z (counterclockwise).
        const double vx = speeds.vx_mps - speeds.omega_rad_per_s * y;
        const double vy = speeds.vy_mps + speeds.omega_rad_per_s * x;
        states[i] = ModuleState{std::hypot(vx, vy), std::atan2(vy, vx)};
    }
    return states;
}

ChassisSpeeds SwerveKinematics::to_chassis_speeds(const std::array<ModuleState, 4> &states) const {
    // Each module contributes two rows to the linear system relating
    // chassis speeds (vx, vy, omega) to that module's (vx_i, vy_i):
    //   [1  0  -y_i] [vx]      [vx_i]
    //   [0  1   x_i] [vy]  ==  [vy_i]
    //                [omega]
    // Stacked over all 4 modules this is overdetermined (8 equations, 3
    // unknowns); solve the least-squares fit via the normal equations
    // (A^T A) x = A^T b, built up directly rather than forming A explicitly.
    std::array<std::array<double, 3>, 3> ata{};
    std::array<double, 3> atb{};
    for (std::size_t i = 0; i < module_positions_.size(); ++i) {
        const double x = module_positions_[i].x_m;
        const double y = module_positions_[i].y_m;
        const double vx = states[i].speed_mps * std::cos(states[i].angle_rad);
        const double vy = states[i].speed_mps * std::sin(states[i].angle_rad);

        ata[0][0] += 1.0;
        ata[0][2] += -y;
        ata[2][0] += -y;
        ata[2][2] += y * y;
        atb[0] += vx;
        atb[2] += -y * vx;

        ata[1][1] += 1.0;
        ata[1][2] += x;
        ata[2][1] += x;
        ata[2][2] += x * x;
        atb[1] += vy;
        atb[2] += x * vy;
    }
    const std::array<double, 3> solved = solve3x3(ata, atb);
    return ChassisSpeeds{solved[0], solved[1], solved[2]};
}

ModuleState SwerveKinematics::optimize(const ModuleState &desired, double current_angle_rad) {
    const double delta = std::remainder(desired.angle_rad - current_angle_rad, 2.0 * M_PI);
    if (std::abs(delta) > M_PI / 2.0) {
        const double flipped_angle = std::remainder(desired.angle_rad + M_PI, 2.0 * M_PI);
        return ModuleState{-desired.speed_mps, flipped_angle};
    }
    return desired;
}

}  // namespace kinematics
