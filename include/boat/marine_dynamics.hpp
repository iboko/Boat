#pragma once
#include "boat/ins.hpp"

namespace boat::marine {

// Experimental planar 3-DOF surge/sway/yaw model (not calibrated to BlueBoat).
// NED positions [N,E]; body axes [forward, starboard]. Positive yaw clockwise.
struct MarineParameters {
    double mass_kg{65.0};
    double yaw_inertia_kgm2{28.0};
    double surge_linear_drag_ns_per_m{20.0};
    double sway_linear_drag_ns_per_m{42.0};
    double surge_quadratic_drag_kg_per_m{13.0};
    double sway_quadratic_drag_kg_per_m{24.0};
    double yaw_drag_nms_per_rad{20.0};
    double wind_force_gain{0.12}; // N / (m/s)^2 (simplified windage)
    double wave_force_n{2.0};
    double wave_yaw_moment_nm{1.0};
    double wave_frequency_radps{0.95};
    double maximum_thrust_n{120.0};
    double maximum_yaw_moment_nm{50.0};
};
struct MarineEnvironment {
    ins::Vec3 water_current_ned_mps{};
    ins::Vec3 wind_ned_mps{};
};
struct MarineControl {
    double forward_thrust_n{};
    double yaw_moment_nm{};
};
struct MarineState {
    double time_s{};
    ins::Vec3 position_ned_m{};
    ins::Vec3 velocity_ground_ned_mps{};
    ins::Vec3 velocity_through_water_body_mps{};
    double heading_rad{};
    double yaw_rate_radps{};
};

class MarineThreeDof {
public:
    explicit MarineThreeDof(MarineParameters parameters = {},
                            MarineEnvironment environment = {});
    void reset();
    bool step(double dt_s, MarineControl command);
    const MarineState& state() const { return state_; }
    const MarineParameters& parameters() const { return parameters_; }
    const MarineEnvironment& environment() const { return environment_; }

private:
    MarineParameters parameters_;
    MarineEnvironment environment_;
    MarineState state_{};
};
} // namespace boat::marine
