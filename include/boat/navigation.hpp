#pragma once

#include <cstddef>
#include <limits>
#include <string>
#include <vector>

namespace boat {

inline constexpr double kPi = 3.14159265358979323846;
double wrapAngle(double radians);

enum class FixSource { Gnss, ShoreRadar, ShoreLidar, SurveyedBeacon };

// Local navigation reference: north/east in metres, yaw clockwise from north.
// This is a level-plane research model, NOT a full strapdown INS.
struct Imu2D {
    double time_s{};
    double yaw_rate_rad_s{};
    double forward_accel_mps2{};
    double right_accel_mps2{};
    // Raw accelerometer outputs CANNOT be used without 3D attitude/gravity compensation.
    bool gravity_compensated_and_levelled{false};
};

struct PositionFix {
    double time_s{};
    double north_m{};
    double east_m{};
    double sigma_m{1.0};
    FixSource source{FixSource::Gnss};
    bool valid{true};
};

struct GroundVelocityFix {
    double time_s{};
    double north_mps{};
    double east_mps{};
    double sigma_mps{0.15};
    bool ground_referenced{false}; // water-relative log MUST NOT set this true.
    bool valid{true};
};

struct NavigationState {
    bool initialized{false};
    double time_s{};
    double north_m{};
    double east_m{};
    double north_mps{};
    double east_mps{};
    double yaw_rad{};
    double pos_variance_m2{1.0};
    double vel_variance_m2ps2{0.25};
    double yaw_variance_rad2{0.01};
    double last_absolute_fix_s{-std::numeric_limits<double>::infinity()};
    bool last_fix_was_gnss{false};
};

struct NoiseConfig {
    double accelerometer_sigma_mps2{0.06};
    double gyro_sigma_radps{0.006};
    // Conservative drift floor for unmodelled bias, heading and current.
    double position_variance_growth_m2_per_s{0.035};
    double velocity_variance_growth_m2ps2_per_s{0.004};
    double fix_nis_gate{11.83}; // 2 DoF chi-square, ~99.73%.
};

class PlanarEstimator {
public:
    explicit PlanarEstimator(NoiseConfig noise = {});
    void initialize(double time_s, double north_m, double east_m,
                    double yaw_rad, double north_mps = 0.0, double east_mps = 0.0,
                    double position_sigma_m = 1.0);
    bool predict(const Imu2D& imu);
    bool correctPosition(const PositionFix& fix);
    bool correctGroundVelocity(const GroundVelocityFix& fix);
    const NavigationState& state() const { return state_; }

private:
    NavigationState state_{};
    NoiseConfig noise_{};
};

struct SafetyLimits {
    double max_position_sigma_m{8.0};
    double max_absolute_fix_age_s{120.0};
    double max_state_age_s{0.75};
};

struct SafetyResult {
    bool can_navigate{false};
    std::string reason;
};

SafetyResult checkSafety(const NavigationState& state, double now_s,
                         const SafetyLimits& limits = {});

struct Waypoint {
    double north_m{};
    double east_m{};
    double acceptance_radius_m{5.0};
};

struct Guidance {
    bool active{false};
    bool finished{false};
    double heading_rad{};
    double distance_m{};
    double desired_speed_mps{};
    std::size_t waypoint_index{};
};

class WaypointNavigator {
public:
    explicit WaypointNavigator(std::vector<Waypoint> route, double cruise_speed_mps = 1.5);
    Guidance update(const NavigationState& state, const SafetyResult& safety);
    std::size_t currentIndex() const { return index_; }

private:
    std::vector<Waypoint> route_;
    double cruise_speed_mps_{};
    std::size_t index_{};
};

} // namespace boat
