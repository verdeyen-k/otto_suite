#include "robot/robot_config.hpp"

#include <fstream>
#include <stdexcept>
#include <unordered_map>

namespace robot {

namespace {

std::string trim(const std::string &s) {
    auto begin = s.find_first_not_of(" \t\r");
    if (begin == std::string::npos) return "";
    auto end = s.find_last_not_of(" \t\r");
    return s.substr(begin, end - begin + 1);
}

double require(const std::unordered_map<std::string, std::string> &kv, const std::string &key,
               const std::string &path) {
    auto it = kv.find(key);
    if (it == kv.end()) {
        throw std::runtime_error("robot config '" + path + "': missing required key '" + key + "'");
    }
    try {
        return std::stod(it->second);
    } catch (const std::exception &) {
        throw std::runtime_error("robot config '" + path + "': key '" + key + "' has non-numeric value '" +
                                  it->second + "'");
    }
}

const std::string &require_string(const std::unordered_map<std::string, std::string> &kv, const std::string &key,
                                   const std::string &path) {
    auto it = kv.find(key);
    if (it == kv.end()) {
        throw std::runtime_error("robot config '" + path + "': missing required key '" + key + "'");
    }
    return it->second;
}

char require_axis(const std::unordered_map<std::string, std::string> &kv, const std::string &key,
                   const std::string &path) {
    const std::string &value = require_string(kv, key, path);
    if (value.size() == 1 && (value[0] == 'a' || value[0] == 'A')) return 'a';
    if (value.size() == 1 && (value[0] == 'b' || value[0] == 'B')) return 'b';
    throw std::runtime_error("robot config '" + path + "': key '" + key + "' must be 'a' or 'b', got '" + value +
                              "'");
}

}  // namespace

RobotConfig load_robot_config(const std::string &path) {
    std::ifstream in(path);
    if (!in) {
        throw std::runtime_error("robot config: could not open '" + path + "'");
    }

    std::unordered_map<std::string, std::string> kv;
    std::string line;
    int line_no = 0;
    while (std::getline(in, line)) {
        ++line_no;
        auto hash = line.find('#');
        if (hash != std::string::npos) line = line.substr(0, hash);
        std::string trimmed = trim(line);
        if (trimmed.empty()) continue;
        auto colon = trimmed.find(':');
        if (colon == std::string::npos) {
            throw std::runtime_error("robot config '" + path + "': line " + std::to_string(line_no) +
                                      ": expected 'key: value', got '" + trimmed + "'");
        }
        kv[trim(trimmed.substr(0, colon))] = trim(trimmed.substr(colon + 1));
    }

    RobotConfig cfg;
    cfg.wheelbase_width_m = require(kv, "wheelbase_width_m", path);
    cfg.wheelbase_length_m = require(kv, "wheelbase_length_m", path);
    cfg.wheel_diameter_m = require(kv, "wheel_diameter_m", path);
    cfg.drive_encoder_counts_per_rev = static_cast<int>(require(kv, "drive_encoder_counts_per_rev", path));
    cfg.drive_gear_ratio = require(kv, "drive_gear_ratio", path);
    cfg.max_speed_mps = require(kv, "max_speed_mps", path);
    cfg.max_omega_deg_s = require(kv, "max_omega_deg_s", path);
    cfg.max_steer_rate_deg_s = require(kv, "max_steer_rate_deg_s", path);
    cfg.max_steer_accel_deg_s2 = require(kv, "max_steer_accel_deg_s2", path);
    cfg.max_accel_mps2 = require(kv, "max_accel_mps2", path);
    cfg.max_wheel_speed_rpm = require(kv, "max_wheel_speed_rpm", path);
    cfg.steer_angle_offset_deg[kFrontLeft] = require(kv, "steer_angle_offset_deg_fl", path);
    cfg.steer_angle_offset_deg[kFrontRight] = require(kv, "steer_angle_offset_deg_fr", path);
    cfg.steer_angle_offset_deg[kRearLeft] = require(kv, "steer_angle_offset_deg_rl", path);
    cfg.steer_angle_offset_deg[kRearRight] = require(kv, "steer_angle_offset_deg_rr", path);

    cfg.module_bus_locations[kFrontLeft] = {static_cast<int>(require(kv, "module_steer_slave_fl", path)),
                                             static_cast<int>(require(kv, "module_drive_slave_fl", path)),
                                             require_axis(kv, "module_drive_axis_fl", path)};
    cfg.module_bus_locations[kFrontRight] = {static_cast<int>(require(kv, "module_steer_slave_fr", path)),
                                              static_cast<int>(require(kv, "module_drive_slave_fr", path)),
                                              require_axis(kv, "module_drive_axis_fr", path)};
    cfg.module_bus_locations[kRearLeft] = {static_cast<int>(require(kv, "module_steer_slave_rl", path)),
                                            static_cast<int>(require(kv, "module_drive_slave_rl", path)),
                                            require_axis(kv, "module_drive_axis_rl", path)};
    cfg.module_bus_locations[kRearRight] = {static_cast<int>(require(kv, "module_steer_slave_rr", path)),
                                             static_cast<int>(require(kv, "module_drive_slave_rr", path)),
                                             require_axis(kv, "module_drive_axis_rr", path)};
    return cfg;
}

}  // namespace robot
