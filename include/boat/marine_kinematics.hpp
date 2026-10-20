#pragma once

#include "boat/ins.hpp"

namespace boat::marine {

// Kinematic 2D teaching model, NOT a validated ship hydrodynamic solver.
// Body heading is measured clockwise from North; N/E inertial coordinates.
struct Environment {
    ins::Vec3 current_ned_mps{};   // horizontal ambient current
    ins::Vec3 wind_ned_mps{};      // wind velocity relative ground
    double water_speed_time_constant_s{1.8};
    double wind_leeway_gain_per_s{0.012};
};
struct MotionState {
    double time_s{};
    ins::Vec3 position_ned_m{};
    ins::Vec3 ground_velocity_ned_mps{};
    ins::Vec3 water_velocity_ned_mps{};
    double heading_rad{};
};

class KinematicSurfaceVessel {
public:
    explicit KinematicSurfaceVessel(Environment environment = {});
    void initialize(double time_s, ins::Vec3 position_ned_m, double heading_rad);
    bool advance(double dt_s, double water_speed_command_mps, double yaw_rate_command_radps);
    const MotionState& state() const { return state_; }
    bool initialized() const { return initialized_; }
private:
    Environment environment_{};
    MotionState state_{};
    bool initialized_{false};
};

} // namespace boat::marine
