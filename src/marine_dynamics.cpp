#include "boat/marine_dynamics.hpp"
#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace boat::marine {
namespace {
bool valid(double v){return std::isfinite(v);}
bool horizontal(ins::Vec3 v){
    return valid(v.x)&&valid(v.y)&&valid(v.z)&&v.z==0.0;
}
double resist(double speed,double linear,double quadratic){
    return linear*speed+quadratic*std::abs(speed)*speed;
}
}

MarineThreeDof::MarineThreeDof(MarineParameters p,MarineEnvironment e)
    :parameters_(p),environment_(e) {
    if(!valid(p.mass_kg)||p.mass_kg<=0 ||
       !valid(p.yaw_inertia_kgm2)||p.yaw_inertia_kgm2<=0 ||
       !valid(p.surge_linear_drag_ns_per_m)||p.surge_linear_drag_ns_per_m<0 ||
       !valid(p.sway_linear_drag_ns_per_m)||p.sway_linear_drag_ns_per_m<0 ||
       !valid(p.surge_quadratic_drag_kg_per_m)||p.surge_quadratic_drag_kg_per_m<0 ||
       !valid(p.sway_quadratic_drag_kg_per_m)||p.sway_quadratic_drag_kg_per_m<0 ||
       !valid(p.yaw_drag_nms_per_rad)||p.yaw_drag_nms_per_rad<0 ||
       !valid(p.wind_force_gain)||p.wind_force_gain<0 ||
       !valid(p.wave_force_n)||p.wave_force_n<0 ||
       !valid(p.wave_yaw_moment_nm)||p.wave_yaw_moment_nm<0 ||
       !valid(p.wave_frequency_radps)||p.wave_frequency_radps<0 ||
       !valid(p.maximum_thrust_n)||p.maximum_thrust_n<=0 ||
       !valid(p.maximum_yaw_moment_nm)||p.maximum_yaw_moment_nm<=0 ||
       !horizontal(e.water_current_ned_mps)||!horizontal(e.wind_ned_mps))
        throw std::invalid_argument("nonphysical marine model parameter");
    reset();
}
void MarineThreeDof::reset(){state_={};}

bool MarineThreeDof::step(double dt,MarineControl cmd) {
    if(!valid(dt)||dt<=0.0||dt>0.05 ||
       !valid(cmd.forward_thrust_n)||!valid(cmd.yaw_moment_nm) ||
       std::abs(cmd.forward_thrust_n)>parameters_.maximum_thrust_n ||
       std::abs(cmd.yaw_moment_nm)>parameters_.maximum_yaw_moment_nm)
        return false;
    const auto& p=parameters_;
    const double yaw=state_.heading_rad;
    const double c=std::cos(yaw),s=std::sin(yaw);
    const auto body=state_.velocity_through_water_body_mps;
    const double u=body.x,v=body.y,r=state_.yaw_rate_radps;
    const double vn=c*u-s*v+environment_.water_current_ned_mps.x;
    const double ve=s*u+c*v+environment_.water_current_ned_mps.y;
    const double relative_wind_n=environment_.wind_ned_mps.x-vn;
    const double relative_wind_e=environment_.wind_ned_mps.y-ve;
    const double wind_force_n=p.wind_force_gain*relative_wind_n*
        std::abs(relative_wind_n);
    const double wind_force_e=p.wind_force_gain*relative_wind_e*
        std::abs(relative_wind_e);
    const double wind_x=c*wind_force_n+s*wind_force_e;
    const double wind_y=-s*wind_force_n+c*wind_force_e;
    const double wave_force=p.wave_force_n*
        std::sin(p.wave_frequency_radps*state_.time_s);
    const double wave_torque=p.wave_yaw_moment_nm*
        std::sin(0.67*p.wave_frequency_radps*state_.time_s+0.7);
    // Simplified rigid-body 3-DOF equations:
    // m(du-rv)=X; m(dv+ru)=Y; Iz dr=N.
    const double du=r*v+
        (cmd.forward_thrust_n-resist(u,p.surge_linear_drag_ns_per_m,
                        p.surge_quadratic_drag_kg_per_m)+wind_x)/p.mass_kg;
    const double dv=-r*u+
        (-resist(v,p.sway_linear_drag_ns_per_m,
                p.sway_quadratic_drag_kg_per_m)+wind_y+wave_force)/p.mass_kg;
    const double dr=(cmd.yaw_moment_nm-p.yaw_drag_nms_per_rad*r+
                     wave_torque)/p.yaw_inertia_kgm2;
    const double new_u=u+du*dt,new_v=v+dv*dt,new_r=r+dr*dt;
    const double new_heading=yaw+0.5*(r+new_r)*dt;
    const double cn=std::cos(new_heading),sn=std::sin(new_heading);
    const double new_vn=cn*new_u-sn*new_v+environment_.water_current_ned_mps.x;
    const double new_ve=sn*new_u+cn*new_v+environment_.water_current_ned_mps.y;
    if(!valid(new_vn)||!valid(new_ve)||!valid(new_heading)||
       !valid(new_u)||!valid(new_v)||!valid(new_r))return false;
    state_.position_ned_m.x+=0.5*(vn+new_vn)*dt;
    state_.position_ned_m.y+=0.5*(ve+new_ve)*dt;
    state_.velocity_through_water_body_mps={new_u,new_v,0.0};
    state_.velocity_ground_ned_mps={new_vn,new_ve,0.0};
    state_.heading_rad=new_heading;
    state_.yaw_rate_radps=new_r;
    state_.time_s+=dt;
    return true;
}
} // namespace boat::marine
