#include "boat/navigation.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace boat {
namespace {
constexpr double kMaxStepS = 0.5;
constexpr double kTimestampTolerance = 0.2;
bool finite(double x) { return std::isfinite(x); }
}

double wrapAngle(double angle) {
    return std::atan2(std::sin(angle), std::cos(angle));
}

PlanarEstimator::PlanarEstimator(NoiseConfig noise) : noise_(noise) {
    if (!(noise_.accelerometer_sigma_mps2 > 0.0 && noise_.gyro_sigma_radps > 0.0 &&
          noise_.position_variance_growth_m2_per_s >= 0.0 &&
          noise_.velocity_variance_growth_m2ps2_per_s >= 0.0 && noise_.fix_nis_gate > 0.0)) {
        throw std::invalid_argument("Invalid estimator noise settings");
    }
}

void PlanarEstimator::initialize(double time_s, double north_m, double east_m,
                                 double yaw_rad, double north_mps, double east_mps,
                                 double position_sigma_m) {
    if (!finite(time_s) || !finite(north_m) || !finite(east_m) ||
        !finite(yaw_rad) || !finite(north_mps) || !finite(east_mps) ||
        !finite(position_sigma_m) || position_sigma_m <= 0.0) {
        throw std::invalid_argument("Invalid initial navigation state");
    }
    state_ = {};
    state_.initialized = true;
    state_.time_s = time_s;
    state_.north_m = north_m;
    state_.east_m = east_m;
    state_.north_mps = north_mps;
    state_.east_mps = east_mps;
    state_.yaw_rad = wrapAngle(yaw_rad);
    state_.pos_variance_m2 = position_sigma_m * position_sigma_m;
    // Explicit external initialization is required; no fabricated absolute fix.
    state_.last_absolute_fix_s = -std::numeric_limits<double>::infinity();
}

bool PlanarEstimator::predict(const Imu2D& imu) {
    if (!state_.initialized || !imu.gravity_compensated_and_levelled ||
        !finite(imu.time_s) || !finite(imu.yaw_rate_rad_s) ||
        !finite(imu.forward_accel_mps2) || !finite(imu.right_accel_mps2)) return false;
    const double dt = imu.time_s - state_.time_s;
    if (!(dt > 0.0 && dt <= kMaxStepS)) return false;
    const double yaw_mid = state_.yaw_rad + 0.5 * imu.yaw_rate_rad_s * dt;
    const double c = std::cos(yaw_mid), s = std::sin(yaw_mid);
    const double an = c * imu.forward_accel_mps2 - s * imu.right_accel_mps2;
    const double ae = s * imu.forward_accel_mps2 + c * imu.right_accel_mps2;
    state_.north_m += state_.north_mps * dt + 0.5 * an * dt * dt;
    state_.east_m += state_.east_mps * dt + 0.5 * ae * dt * dt;
    state_.north_mps += an * dt;
    state_.east_mps += ae * dt;
    state_.yaw_rad = wrapAngle(state_.yaw_rad + imu.yaw_rate_rad_s * dt);
    const double accel_var = noise_.accelerometer_sigma_mps2 * noise_.accelerometer_sigma_mps2;
    const double speed_sq = state_.north_mps * state_.north_mps + state_.east_mps * state_.east_mps;
    state_.pos_variance_m2 += dt * dt * state_.vel_variance_m2ps2 +
        0.25 * accel_var * dt * dt * dt * dt +
        speed_sq * state_.yaw_variance_rad2 * dt * dt +
        noise_.position_variance_growth_m2_per_s * dt;
    state_.vel_variance_m2ps2 += accel_var * dt * dt +
        noise_.velocity_variance_growth_m2ps2_per_s * dt;
    state_.yaw_variance_rad2 += noise_.gyro_sigma_radps * noise_.gyro_sigma_radps * dt * dt;
    state_.time_s = imu.time_s;
    return true;
}

bool PlanarEstimator::correctPosition(const PositionFix& fix) {
    if (!state_.initialized || !fix.valid || !finite(fix.time_s) ||
        !finite(fix.north_m) || !finite(fix.east_m) || !finite(fix.sigma_m) ||
        !(fix.sigma_m > 0.0) || std::abs(fix.time_s - state_.time_s) > kTimestampTolerance) return false;
    const double r = fix.sigma_m * fix.sigma_m;
    const double innovation_variance = state_.pos_variance_m2 + r;
    const double dn = fix.north_m - state_.north_m;
    const double de = fix.east_m - state_.east_m;
    if ((dn * dn + de * de) / innovation_variance > noise_.fix_nis_gate) return false;
    const double k = state_.pos_variance_m2 / innovation_variance;
    state_.north_m += k * dn;
    state_.east_m += k * de;
    state_.pos_variance_m2 = std::max(1e-8, (1.0 - k) * state_.pos_variance_m2);
    state_.last_absolute_fix_s = fix.time_s;
    state_.last_fix_was_gnss = (fix.source == FixSource::Gnss);
    return true;
}

bool PlanarEstimator::correctGroundVelocity(const GroundVelocityFix& fix) {
    if (!state_.initialized || !fix.valid || !fix.ground_referenced ||
        !finite(fix.time_s) || !finite(fix.north_mps) || !finite(fix.east_mps) ||
        !finite(fix.sigma_mps) || !(fix.sigma_mps > 0.0) ||
        std::abs(fix.time_s - state_.time_s) > kTimestampTolerance) return false;
    const double r = fix.sigma_mps * fix.sigma_mps;
    const double innovation_variance = state_.vel_variance_m2ps2 + r;
    const double dn = fix.north_mps - state_.north_mps;
    const double de = fix.east_mps - state_.east_mps;
    if ((dn * dn + de * de) / innovation_variance > noise_.fix_nis_gate) return false;
    const double k = state_.vel_variance_m2ps2 / innovation_variance;
    state_.north_mps += k * dn;
    state_.east_mps += k * de;
    state_.vel_variance_m2ps2 = std::max(1e-8, (1.0 - k) * state_.vel_variance_m2ps2);
    return true;
}

SafetyResult checkSafety(const NavigationState& s, double now_s, const SafetyLimits& l) {
    if (!s.initialized) return {false, "not initialized"};
    if (!finite(now_s) || now_s < s.time_s || now_s - s.time_s > l.max_state_age_s)
        return {false, "state timestamp expired"};
    if (!finite(s.north_m) || !finite(s.east_m) || !finite(s.pos_variance_m2) ||
        s.pos_variance_m2 < 0.0) return {false, "invalid navigation state"};
    if (!finite(s.last_absolute_fix_s) ||
        now_s - s.last_absolute_fix_s > l.max_absolute_fix_age_s)
        return {false, "no recent independent absolute position fix"};
    if (std::sqrt(s.pos_variance_m2) > l.max_position_sigma_m)
        return {false, "position uncertainty limit exceeded"};
    return {true, "navigation estimate acceptable for research simulation"};
}

WaypointNavigator::WaypointNavigator(std::vector<Waypoint> route, double speed)
    : route_(std::move(route)), cruise_speed_mps_(speed) {
    if (!(speed > 0.0 && finite(speed))) throw std::invalid_argument("Invalid cruise speed");
    for (const auto& w : route_) {
        if (!finite(w.north_m) || !finite(w.east_m) ||
            !finite(w.acceptance_radius_m) || w.acceptance_radius_m <= 0.0)
            throw std::invalid_argument("Invalid waypoint");
    }
}

Guidance WaypointNavigator::update(const NavigationState& s, const SafetyResult& safety) {
    if (!safety.can_navigate || !s.initialized) return {};
    while (index_ < route_.size()) {
        const auto& w = route_[index_];
        const double dn = w.north_m - s.north_m;
        const double de = w.east_m - s.east_m;
        const double d = std::hypot(dn, de);
        // With poor positional certainty do NOT claim waypoint arrival.
        if (d <= w.acceptance_radius_m &&
            2.0 * std::sqrt(s.pos_variance_m2) <= w.acceptance_radius_m) {
            ++index_;
            continue;
        }
        return {true, false, std::atan2(de, dn), d, cruise_speed_mps_, index_};
    }
    Guidance out{};
    out.finished = true;
    out.waypoint_index = index_;
    return out;
}
} // namespace boat
