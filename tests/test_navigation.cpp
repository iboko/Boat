#include "boat/ardupilot.hpp"
#include "boat/navigation.hpp"

#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>

namespace {
void check(bool cond, const std::string& what) {
    if (!cond) throw std::runtime_error(what);
}
void near(double actual, double expected, double tolerance, const std::string& what) {
    check(std::abs(actual - expected) <= tolerance, what);
}
}

int main() {
    using namespace boat;
    try {
        PlanarEstimator estimator;
        check(!estimator.predict({0.1, 0.0, 0.0, 0.0, true}), "no uninitialized INS");
        estimator.initialize(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0);
        check(!checkSafety(estimator.state(), 0.0).can_navigate, "initial position is not a fix");
        check(estimator.correctPosition({0.0, 0.0, 0.0, 0.5, FixSource::Gnss, true}), "initial fix");
        check(checkSafety(estimator.state(), 0.0).can_navigate, "valid navigation");
        check(!estimator.predict({0.1, 0.0, 2.0, 0.0, false}), "reject raw accelerometer");
        check(estimator.predict({0.1, 0.0, 2.0, 0.0, true}), "predict leveled acceleration");
        near(estimator.state().north_m, 0.01, 1e-10, "inertial displacement");
        near(estimator.state().north_mps, 0.2, 1e-10, "inertial velocity");
        check(!estimator.predict({0.1, 0.0, 0.0, 0.0, true}), "reject duplicate timestamp");
        check(!estimator.predict({2.1, 0.0, 0.0, 0.0, true}), "reject missing IMU epoch");
        check(!estimator.correctPosition({0.1, 1000.0, 0.0, 0.2, FixSource::Gnss, true}),
              "reject GNSS innovation outlier");
        near(estimator.state().north_m, 0.01, 1e-10, "outlier cannot move position");
        check(!estimator.correctGroundVelocity({0.1, 0.0, 0.0, 0.1, false, true}),
              "water log cannot be ground velocity");
        check(estimator.correctGroundVelocity({0.1, 0.4, 0.0, 0.2, true, true}),
              "ground referenced velocity correction");
        check(estimator.correctPosition({0.1, 0.01, 0.0, 0.5, FixSource::ShoreRadar, true}),
              "independent radar correction");
        check(!estimator.state().last_fix_was_gnss, "source identification");
        check(!checkSafety(estimator.state(), 150.0).can_navigate, "stale position unsafe");
        check(!checkSafety(estimator.state(), 2.0).can_navigate, "stale navigation state unsafe");
        check(!checkSafety(estimator.state(), 0.1, {0.01, 120.0, 1.0}).can_navigate,
              "uncertainty limit unsafe");

        WaypointNavigator route({{0.0, 0.0, 3.0}, {15.0, 0.0, 3.0}});
        auto first = route.update(estimator.state(), checkSafety(estimator.state(), 0.1));
        check(first.active && first.waypoint_index == 1, "advance reached waypoint");
        near(first.heading_rad, 0.0, 0.01, "northbound heading");
        check(!route.update(estimator.state(), {false, "fault"}).active,
              "block guidance when navigation unsafe");
        check(!ArduPilotOdometryEncoder::eligible({}), "no synthetic fake 6DoF data");
        check(ArduPilotOdometryEncoder::encode({}).empty(), "invalid MAVLink output suppressed");

        std::cout << "PASS: navigation and safety checks\n";
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "FAIL: " << e.what() << '\n';
        return 1;
    }
}
