#pragma once

#include <cstdint>
#include <vector>

namespace boat {

// External-navigation ODOMETRY is accepted only from a separately VALIDATED
// 6-DoF estimator. The 2D demonstration cannot supply such data.
struct ValidatedOdometry6D {
    double time_s{}; // monotonic time since companion start
    double north_m{}, east_m{}, down_m{};
    double roll_rad{}, pitch_rad{}, yaw_rad{};
    double north_mps{}, east_mps{}, down_mps{};
    double roll_rate_radps{}, pitch_rate_radps{}, yaw_rate_radps{};
    double position_sigma_m{1.0};
    double angular_sigma_rad{0.1};
    std::uint8_t quality{0}; // 1..100; zero must not be transmitted.
    std::uint8_t reset_counter{0};
    bool origin_is_fixed_and_aligned{false};
    bool vertical_state_valid{false};
    bool full_attitude_valid{false};
    bool independent_sensor_corrections_valid{false};
};

class ArduPilotOdometryEncoder {
public:
    // All invalid states are rejected. Empty result also means MAVLink build OFF.
    static bool eligible(const ValidatedOdometry6D& odom);
    static std::vector<std::uint8_t> encode(const ValidatedOdometry6D& odom,
                                             std::uint8_t system_id = 42,
                                             std::uint8_t component_id = 191);
};

} // namespace boat
