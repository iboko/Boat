#include "boat/marine_kinematics.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace boat::marine {
namespace {
bool finite(double x){return std::isfinite(x);}
bool horizontal(ins::Vec3 v){return finite(v.x)&&finite(v.y)&&finite(v.z)&&v.z==0.0;}
ins::Vec3 forward(double heading){return {std::cos(heading),std::sin(heading),0.0};}
}

KinematicSurfaceVessel::KinematicSurfaceVessel(Environment e):environment_(e){
    if(!horizontal(e.current_ned_mps)||!horizontal(e.wind_ned_mps)||
       !finite(e.water_speed_time_constant_s)||
       !(e.water_speed_time_constant_s>0.0)||
       !finite(e.wind_leeway_gain_per_s) ||
       e.wind_leeway_gain_per_s<0.0 || e.wind_leeway_gain_per_s>0.1)
        throw std::invalid_argument("invalid kinematic marine environment");
}

void KinematicSurfaceVessel::initialize(double t,ins::Vec3 p,double yaw){
    if(!finite(t)||!finite(p.x)||!finite(p.y)||!finite(yaw))
        throw std::invalid_argument("invalid kinematic vessel state");
    initialized_=true;
    state_={};
    state_.time_s=t;
    state_.position_ned_m={p.x,p.y,0.0};
    state_.heading_rad=yaw;
    state_.ground_velocity_ned_mps=environment_.current_ned_mps;
}

bool KinematicSurfaceVessel::advance(double dt,double commanded_speed,double commanded_yaw_rate){
    if(!initialized_ || !finite(dt)||dt<=0.0||dt>0.1||
       !finite(commanded_speed)||commanded_speed<0||commanded_speed>8.0||
       !finite(commanded_yaw_rate)||std::abs(commanded_yaw_rate)>1.0)return false;
    const double yaw_mid=state_.heading_rad+0.5*commanded_yaw_rate*dt;
    const auto desired=forward(yaw_mid)*commanded_speed;
    // Linearized surge relaxation and wind leeway in the N/E frame.
    // Not a displacement-hull drag, rudder lift or maneuvering dynamics model.
    const auto wind_relative=environment_.wind_ned_mps-
                            state_.ground_velocity_ned_mps;
    const auto acceleration=(desired-state_.water_velocity_ned_mps)/
                    environment_.water_speed_time_constant_s+
                    wind_relative*environment_.wind_leeway_gain_per_s;
    const auto previous=state_.ground_velocity_ned_mps;
    state_.water_velocity_ned_mps=state_.water_velocity_ned_mps+acceleration*dt;
    state_.ground_velocity_ned_mps=state_.water_velocity_ned_mps+
                                    environment_.current_ned_mps;
    state_.position_ned_m=state_.position_ned_m+
                          (previous+state_.ground_velocity_ned_mps)*(0.5*dt);
    state_.heading_rad+=commanded_yaw_rate*dt;
    state_.time_s+=dt;
    return true;
}
} // namespace boat::marine
