#include "boat/marine_windowed.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace boat::marine {
namespace {

bool valid(ins::Vec3 v) {
    return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
}
bool valid(const MarineState& s) {
    return std::isfinite(s.time_s) && s.time_s>=0.0 &&
        valid(s.position_ned_m) && valid(s.velocity_ground_ned_mps) &&
        valid(s.velocity_through_water_body_mps) &&
        std::isfinite(s.heading_rad) && std::isfinite(s.yaw_rate_radps);
}

struct Regression {
    double a{},b{},c{},d{},e{},yy{};
    void add(double x0,double x1,double y) {
        a+=x0*x0;b+=x0*x1;c+=x1*x1;
        d+=x0*y;e+=x1*y;yy+=y*y;
    }
    struct Fit {double alpha,beta,rank,rmse;};
    Fit solve(std::size_t n) const {
        if(n<4 || !std::isfinite(a) || !std::isfinite(b) ||
           !std::isfinite(c) || !std::isfinite(d) || !std::isfinite(e) ||
           !std::isfinite(yy) || a<=1e-9 || c<=1e-9)
            throw std::invalid_argument("insufficient windowed excitation");
        const double det=a*c-b*b;
        const double rank=det/(a*c);
        if(!std::isfinite(rank) || rank<1e-6)
            throw std::invalid_argument("singular windowed identification");
        const double alpha=(d*c-e*b)/det;
        const double beta=(a*e-b*d)/det;
        const double err=yy-2.0*(alpha*d+beta*e)+alpha*alpha*a+
            2.0*alpha*beta*b+beta*beta*c;
        const double rmse=std::sqrt(std::max(0.0,err/n));
        if(!std::isfinite(alpha)||!std::isfinite(beta)||!std::isfinite(rmse))
            throw std::invalid_argument("nonfinite windowed fit");
        return {alpha,beta,rank,rmse};
    }
};

} // namespace

WindowedIdentificationResult identifyMarineParametersWindowed(
    const std::vector<IdentificationSample>& samples,
    std::size_t width,MarineParameters known) {
    (void)MarineThreeDof(known,{});
    if(width<2 || width>200 || samples.size()/width<4)
        throw std::invalid_argument("insufficient data for integration windows");
    const std::size_t n=samples.size()/width;
    Regression surge,yaw;
    for(std::size_t window=0;window<n;++window) {
        const std::size_t start=window*width;
        const auto& first=samples[start].before;
        const auto& last=samples[start+width-1].after;
        double s0=0.0,s1=0.0,rot=0.0,w0=0.0,w1=0.0;
        for(std::size_t i=start;i<start+width;++i) {
            const auto& sample=samples[i];
            const auto& a=sample.before;
            const double dt=sample.after.time_s-a.time_s;
            if(!valid(a) || !valid(sample.after) ||
               !std::isfinite(dt) || dt<=0.0 || dt>0.05 ||
               !std::isfinite(sample.applied.forward_thrust_n) ||
               !std::isfinite(sample.applied.yaw_moment_nm) ||
               std::abs(sample.applied.forward_thrust_n)>known.maximum_thrust_n ||
               std::abs(sample.applied.yaw_moment_nm)>known.maximum_yaw_moment_nm ||
               !valid(sample.environment.water_current_ned_mps) ||
               !valid(sample.environment.wind_ned_mps) ||
               sample.environment.water_current_ned_mps.z!=0.0 ||
               sample.environment.wind_ned_mps.z!=0.0)
                throw std::invalid_argument("invalid windowed marine record");
            if(i>start) {
                const auto& preceding=samples[i-1].after;
                // Require the exact same observation to be used across the
                // boundary. Otherwise sums of differences do not telescope.
                if(std::abs(a.time_s-preceding.time_s)>1e-9 ||
                   std::abs(a.velocity_through_water_body_mps.x-
                            preceding.velocity_through_water_body_mps.x)>1e-9 ||
                   std::abs(a.velocity_through_water_body_mps.y-
                            preceding.velocity_through_water_body_mps.y)>1e-9 ||
                   std::abs(a.yaw_rate_radps-preceding.yaw_rate_radps)>1e-9)
                    throw std::invalid_argument("discontinuous windowed measurements");
            }
            const double u=a.velocity_through_water_body_mps.x;
            const double v=a.velocity_through_water_body_mps.y;
            const double r=a.yaw_rate_radps;
            const double cs=std::cos(a.heading_rad),sn=std::sin(a.heading_rad);
            const double gn=cs*u-sn*v+sample.environment.water_current_ned_mps.x;
            const double ge=sn*u+cs*v+sample.environment.water_current_ned_mps.y;
            const double rn=sample.environment.wind_ned_mps.x-gn;
            const double re=sample.environment.wind_ned_mps.y-ge;
            const double wind_x=known.wind_force_gain*
                (cs*rn*std::abs(rn)+sn*re*std::abs(re));
            const double force=sample.applied.forward_thrust_n-
                known.surge_quadratic_drag_kg_per_m*std::abs(u)*u+wind_x;
            const double wave=known.wave_yaw_moment_nm*
                std::sin(0.67*known.wave_frequency_radps*a.time_s+0.7);
            s0+=dt*force;
            s1-=dt*u;
            rot+=dt*r*v;
            w0+=dt*(sample.applied.yaw_moment_nm+wave);
            w1-=dt*r;
        }
        const double target_u=last.velocity_through_water_body_mps.x-
            first.velocity_through_water_body_mps.x-rot;
        const double target_r=last.yaw_rate_radps-first.yaw_rate_radps;
        surge.add(s0,s1,target_u);
        yaw.add(w0,w1,target_r);
    }
    const auto s=surge.solve(n),w=yaw.solve(n);
    if(s.alpha<=0.0 || w.alpha<=0.0 || s.beta< -1e-10 || w.beta< -1e-10)
        throw std::invalid_argument("nonphysical integrated marine estimate");
    known.mass_kg=1.0/s.alpha;
    known.surge_linear_drag_ns_per_m=std::max(0.0,s.beta/s.alpha);
    known.yaw_inertia_kgm2=1.0/w.alpha;
    known.yaw_drag_nms_per_rad=std::max(0.0,w.beta/w.alpha);
    if(!std::isfinite(known.mass_kg) ||
       !std::isfinite(known.surge_linear_drag_ns_per_m) ||
       !std::isfinite(known.yaw_inertia_kgm2) ||
       !std::isfinite(known.yaw_drag_nms_per_rad))
        throw std::invalid_argument("nonfinite integrated marine estimate");
    (void)MarineThreeDof(known,{});
    return {known,n,n*width,s.rank,w.rank,s.rmse,w.rmse};
}

} // namespace boat::marine
