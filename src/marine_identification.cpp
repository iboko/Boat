#include "boat/marine_identification.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace boat::marine {
namespace {

bool finite(ins::Vec3 v) {
    return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
}
bool finiteState(const MarineState& s) {
    return std::isfinite(s.time_s) && s.time_s>=0.0 &&
        finite(s.position_ned_m) && finite(s.velocity_ground_ned_mps) &&
        finite(s.velocity_through_water_body_mps) &&
        std::isfinite(s.heading_rad) && std::isfinite(s.yaw_rate_radps);
}

struct Regression {
    double a{}, b{}, c{}, d{}, e{}, yy{};
    void add(double x0,double x1,double y) {
        a+=x0*x0;
        b+=x0*x1;
        c+=x1*x1;
        d+=x0*y;
        e+=x1*y;
        yy+=y*y;
    }
    struct Fit {double first,second,excitation,rmse;};
    Fit solve(std::size_t n) const {
        if(n<4 || !std::isfinite(a) || !std::isfinite(c) ||
           !std::isfinite(d) || !std::isfinite(e) || !std::isfinite(yy) ||
           a<=1e-9 || c<=1e-9)
            throw std::invalid_argument("insufficient marine excitation");
        const double determinant=a*c-b*b;
        const double normalized=determinant/(a*c);
        if(!std::isfinite(normalized) || normalized<1e-6)
            throw std::invalid_argument("unidentifiable marine coefficients");
        const double first=(d*c-e*b)/determinant;
        const double second=(a*e-b*d)/determinant;
        const double square=yy-2.0*(first*d+second*e)+
            first*first*a+2.0*first*second*b+second*second*c;
        const double rmse=std::sqrt(std::max(0.0,square/static_cast<double>(n)));
        if(!std::isfinite(first)||!std::isfinite(second)||!std::isfinite(rmse))
            throw std::invalid_argument("nonfinite marine identification");
        return {first,second,normalized,rmse};
    }
};

} // namespace

IdentificationResult identifyMarineParameters(
    const std::vector<IdentificationSample>& samples,MarineParameters known) {
    // Reuse the forward model's validation of all declared fixed coefficients.
    (void)MarineThreeDof(known,{});
    if(samples.size()<4)
        throw std::invalid_argument("too few marine samples");
    Regression surge,yaw;
    for(const auto& sample:samples) {
        const auto& a=sample.before;
        const auto& z=sample.after;
        const double dt=z.time_s-a.time_s;
        if(!finiteState(a) || !finiteState(z) || !std::isfinite(dt) ||
           dt<=0.0 || dt>0.05 ||
           !std::isfinite(sample.applied.forward_thrust_n) ||
           !std::isfinite(sample.applied.yaw_moment_nm) ||
           std::abs(sample.applied.forward_thrust_n)>known.maximum_thrust_n ||
           std::abs(sample.applied.yaw_moment_nm)>known.maximum_yaw_moment_nm ||
           !finite(sample.environment.wind_ned_mps) ||
           !finite(sample.environment.water_current_ned_mps) ||
           sample.environment.wind_ned_mps.z!=0.0 ||
           sample.environment.water_current_ned_mps.z!=0.0)
            throw std::invalid_argument("invalid timed marine observation");

        const double u=a.velocity_through_water_body_mps.x;
        const double v=a.velocity_through_water_body_mps.y;
        const double r=a.yaw_rate_radps;
        const double cs=std::cos(a.heading_rad),sn=std::sin(a.heading_rad);
        const double ground_n=cs*u-sn*v+sample.environment.water_current_ned_mps.x;
        const double ground_e=sn*u+cs*v+sample.environment.water_current_ned_mps.y;
        const double rel_n=sample.environment.wind_ned_mps.x-ground_n;
        const double rel_e=sample.environment.wind_ned_mps.y-ground_e;
        const double force_n=known.wind_force_gain*rel_n*std::abs(rel_n);
        const double force_e=known.wind_force_gain*rel_e*std::abs(rel_e);
        const double wind_x=cs*force_n+sn*force_e;
        const double force=sample.applied.forward_thrust_n-
            known.surge_quadratic_drag_kg_per_m*std::abs(u)*u+wind_x;
        const double surge_acceleration=
            (z.velocity_through_water_body_mps.x-u)/dt-r*v;
        surge.add(force,-u,surge_acceleration);

        const double wave_torque=known.wave_yaw_moment_nm*
            std::sin(0.67*known.wave_frequency_radps*a.time_s+0.7);
        const double torque=sample.applied.yaw_moment_nm+wave_torque;
        const double yaw_acceleration=(z.yaw_rate_radps-r)/dt;
        yaw.add(torque,-r,yaw_acceleration);
    }
    const auto s=surge.solve(samples.size()),w=yaw.solve(samples.size());
    if(s.first<=0.0 || w.first<=0.0 || s.second< -1e-10 ||
       w.second< -1e-10)
        throw std::invalid_argument("nonphysical identified marine coefficients");
    known.mass_kg=1.0/s.first;
    known.surge_linear_drag_ns_per_m=std::max(0.0,s.second/s.first);
    known.yaw_inertia_kgm2=1.0/w.first;
    known.yaw_drag_nms_per_rad=std::max(0.0,w.second/w.first);
    if(!std::isfinite(known.mass_kg) ||
       !std::isfinite(known.surge_linear_drag_ns_per_m) ||
       !std::isfinite(known.yaw_inertia_kgm2) ||
       !std::isfinite(known.yaw_drag_nms_per_rad))
        throw std::invalid_argument("unbounded identified marine coefficients");
    (void)MarineThreeDof(known,{});
    return {known,samples.size(),s.excitation,w.excitation,s.rmse,w.rmse};
}

} // namespace boat::marine
