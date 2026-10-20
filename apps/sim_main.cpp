#include "boat/navigation.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <vector>

int main() {
    using namespace boat;
    // Deterministic synthetic scenario only; no connection to propellers or autopilot.
    PlanarEstimator estimator;
    estimator.initialize(0.0, 0.0, 0.0, 0.0);
    estimator.correctPosition({0.0, 0.0, 0.0, 0.8, FixSource::Gnss, true});
    WaypointNavigator route({{35.0, 0.0, 5.0}, {35.0, 35.0, 5.0}, {0.0, 35.0, 5.0}});

    double north = 0.0, east = 0.0, yaw = 0.0, forward_speed = 0.0;
    constexpr double dt = 0.05;
    constexpr double current_north = 0.04, current_east = -0.02;
    std::cout << "time_s,true_n,true_e,estimated_n,estimated_e,pos_error_m,pos_sigma_m,"
                 "mode,waypoint\n";
    for (int i = 1; i <= 6000; ++i) {
        const double t = i * dt;
        const auto guidance = route.update(estimator.state(), checkSafety(estimator.state(), (i-1)*dt));
        const double yaw_rate = guidance.active ?
            std::clamp(1.2 * wrapAngle(guidance.heading_rad - estimator.state().yaw_rad), -0.45, 0.45)
            : 0.0;
        const double desired_speed = guidance.active ? guidance.desired_speed_mps : 0.0;
        const double f_accel = std::clamp((desired_speed - forward_speed) * 1.0, -0.25, 0.25);
        const double midpoint_yaw = yaw + 0.5 * yaw_rate * dt;
        const double speed_mid = forward_speed + 0.5 * f_accel * dt;
        north += (speed_mid * std::cos(midpoint_yaw) + current_north) * dt;
        east += (speed_mid * std::sin(midpoint_yaw) + current_east) * dt;
        // Centripetal component in body frame: u * yaw_rate, right positive.
        const double right_accel = speed_mid * yaw_rate;
        yaw = wrapAngle(yaw + yaw_rate * dt);
        forward_speed += f_accel * dt;
        if (!estimator.predict({t, yaw_rate, f_accel, right_accel, true})) return 2;

        // Simulated GNSS is unavailable from 20 to 160 seconds.
        // Radar position is independent and available only when landmarks exist.
        if (i % 20 == 0) {
            if (t <= 20.0 || t >= 160.0) {
                estimator.correctPosition({t, north + 0.1, east - 0.1, 1.0, FixSource::Gnss, true});
            }
            if (t > 20.0 && t < 160.0 && i % 40 == 0) {
                estimator.correctPosition({t, north + 0.2, east - 0.2, 1.5, FixSource::ShoreRadar, true});
            }
            // Bottom-track Doppler velocity; valid only when seabed is tracked.
            const double vn = forward_speed * std::cos(yaw) + current_north;
            const double ve = forward_speed * std::sin(yaw) + current_east;
            estimator.correctGroundVelocity({t, vn, ve, 0.1, true, true});
        }
        if (i % 20 == 0) {
            const auto& s = estimator.state();
            const auto safe = checkSafety(s, t);
            std::cout << std::fixed << std::setprecision(3)
                << t << ',' << north << ',' << east << ',' << s.north_m << ',' << s.east_m
                << ',' << std::hypot(north - s.north_m, east - s.east_m)
                << ',' << std::sqrt(s.pos_variance_m2) << ','
                << (safe.can_navigate ? (t > 20.0 && t < 160.0 ? "DENIED_GNSS" : "GNSS_OK") : "HOLD")
                << ',' << route.currentIndex() << '\n';
        }
    }
}
