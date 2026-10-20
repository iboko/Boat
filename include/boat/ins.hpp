#pragma once

#include "boat/navigation.hpp"

#include <array>
#include <cstddef>
#include <limits>
#include <string>

namespace boat::ins {

// Local, flat-earth North-East-Down mechanization. FRD body axes.
// Research ESKF only: Earth rotation, transport rate, datum and tide are omitted.
struct Vec3 {
    double x{}, y{}, z{};
};
Vec3 operator+(Vec3 a, Vec3 b);
Vec3 operator-(Vec3 a, Vec3 b);
Vec3 operator*(Vec3 a, double scale);
Vec3 operator/(Vec3 a, double scale);
double norm(Vec3 a);
double dot(Vec3 a, Vec3 b);
Vec3 cross(Vec3 a, Vec3 b);

struct Quaternion {
    double w{1.0}, x{}, y{}, z{};
};
Quaternion multiply(Quaternion a, Quaternion b);
Quaternion normalize(Quaternion q);
Quaternion expRotation(Vec3 radians);
Vec3 rotate(Quaternion body_to_ned, Vec3 body);
Vec3 rotateInverse(Quaternion body_to_ned, Vec3 ned);

struct ImuSample {
    double time_s{};
    Vec3 specific_force_frd_mps2{}; // raw calibrated specific force, includes gravity
    Vec3 angular_rate_frd_radps{};
    bool calibrated{false};
};

// Absolute position is stated in the same fixed, surveyed local NED datum.
enum class AbsoluteSource { Gnss, ShoreRadar, ShoreLidar, SurveyedBeacon };
struct AbsoluteFix {
    double time_s{};
    Vec3 position_ned_m{};
    double horizontal_sigma_m{1.5};
    double vertical_sigma_m{2.0};
    AbsoluteSource source{AbsoluteSource::Gnss};
    bool valid{false};
    bool independently_georeferenced{false}; // false if derived from GNSS in another module
};

struct GroundVelocity {
    double time_s{};
    Vec3 velocity_ned_mps{};
    double horizontal_sigma_mps{0.2};
    double vertical_sigma_mps{0.3};
    bool ground_referenced{false}; // water log is NOT ground-referenced
    bool valid{false};
};

using Covariance15 = std::array<double, 225>;
// Error state [position(3), ground velocity(3), right-body attitude(3),
//              accel bias(3), gyro bias(3)].
// Covariance is row-major, dimensions 15 x 15.
struct State {
    bool initialized{false};
    bool origin_fixed_and_aligned{false};
    double time_s{};
    Vec3 position_ned_m{};
    Vec3 velocity_ned_mps{};
    Quaternion body_to_ned{};
    Vec3 accel_bias_frd_mps2{};
    Vec3 gyro_bias_frd_radps{};
    Covariance15 covariance{};
    double last_absolute_fix_s{-std::numeric_limits<double>::infinity()};
    double last_non_gnss_fix_s{-std::numeric_limits<double>::infinity()};
    double last_gnss_fix_s{-std::numeric_limits<double>::infinity()};
    unsigned accepted_fixes{};
    unsigned rejected_fixes{};
};

struct Configuration {
    double max_imu_dt_s{0.05};
    double max_measurement_age_s{0.03}; // out-of-sequence measurements not implemented
    double gravity_ned_mps2{9.80665};
    double accel_white_noise_mps2_per_sqrt_hz{0.08};
    double gyro_white_noise_radps_per_sqrt_hz{0.004};
    double accel_bias_walk_mps3_per_sqrt_hz{0.0005};
    double gyro_bias_walk_radps2_per_sqrt_hz{0.00003};
    double max_position_nis{16.27}; // 3 DoF 99.9% approximate threshold
    double max_velocity_nis{16.27};
    double initial_position_sigma_m{4.0};
    double initial_velocity_sigma_mps{1.0};
    double initial_attitude_sigma_rad{0.08};
    double initial_accel_bias_sigma_mps2{0.2};
    double initial_gyro_bias_sigma_radps{0.02};
};

class ErrorStateIns {
public:
    explicit ErrorStateIns(Configuration cfg = {});

    // Explicit initial attitude is mandatory. Does not claim a fresh absolute fix.
    void initialize(double time_s, Vec3 position_ned_m, Vec3 velocity_ned_mps,
                    Quaternion body_to_ned, bool origin_fixed_and_aligned);
    bool propagate(const ImuSample& imu);
    bool correctPosition(const AbsoluteFix& fix);
    bool correctGroundVelocity(const GroundVelocity& observation);
    const State& state() const { return state_; }
    static double horizontalSigmaM(const State& state);
    static double maximumCovarianceAsymmetry(const State& state);

private:
    Configuration cfg_;
    State state_{};
    bool measurementUpdate(Vec3 residual, const std::array<double, 3>& variances,
                           int state_offset, double nis_gate);
};

struct IntegrityPolicy {
    double max_horizontal_sigma_m{6.0};
    double max_fix_age_s{90.0};
    double max_state_age_s{0.2};
    double max_independent_fix_age_during_gnss_denial_s{30.0};
};
struct Integrity {
    bool valid{false};
    std::string reason;
};
Integrity assess(const State& state, double now_s, bool gnss_denied,
                 const IntegrityPolicy& policy = {});

// Translation only. No direct motor commands and no automatic ArduPilot publish.
NavigationState makePlanarSnapshot(const State& state);

} // namespace boat::ins
