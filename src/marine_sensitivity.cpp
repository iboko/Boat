#include "boat/marine_sensitivity.hpp"

#include <array>
#include <cmath>
#include <stdexcept>

namespace boat::marine {
namespace {

struct Trial {
    double drift_m{};
    double yaw_change_rad{};
};

Trial measure(MarineParameters parameters, MarineEnvironment environment) {
    MarineThreeDof vessel(parameters,environment);
    constexpr double dt=0.02;
    // Known open-loop input is identical for every perturbation. A symmetric
    // command would not excite yaw inertia and lateral drag meaningfully.
    for(int i=0;i<250;++i)
        if(!vessel.stepTwinPropulsors(dt,{35.0,25.0}))
            throw std::runtime_error("sensitivity: powered motion failed");
    const auto neutral_position=vessel.state().position_ned_m;
    const double neutral_heading=vessel.state().heading_rad;
    for(int i=0;i<1000;++i)
        if(!vessel.stepTwinPropulsors(dt,{0.0,0.0}))
            throw std::runtime_error("sensitivity: neutral motion failed");
    const auto displacement=vessel.state().position_ned_m-neutral_position;
    const double drift=std::hypot(displacement.x,displacement.y);
    const double yaw=std::atan2(
        std::sin(vessel.state().heading_rad-neutral_heading),
        std::cos(vessel.state().heading_rad-neutral_heading));
    if(!std::isfinite(drift) || !std::isfinite(yaw))
        throw std::runtime_error("sensitivity: nonfinite output");
    return {drift,yaw};
}

struct Profile {
    const char* name;
    MarineEnvironment environment;
};
struct Factor {
    const char* name;
    double MarineParameters::*field;
};

constexpr std::array<Factor,6> factors{{
    {"mass_kg",&MarineParameters::mass_kg},
    {"surge_linear_drag_ns_per_m",&MarineParameters::surge_linear_drag_ns_per_m},
    {"sway_linear_drag_ns_per_m",&MarineParameters::sway_linear_drag_ns_per_m},
    {"yaw_drag_nms_per_rad",&MarineParameters::yaw_drag_nms_per_rad},
    {"wind_force_gain",&MarineParameters::wind_force_gain},
    {"propulsor_half_spacing_m",&MarineParameters::propulsor_half_spacing_m},
}};

} // namespace

std::vector<SensitivityRow> evaluateSensitivity(double h) {
    if(!std::isfinite(h) || h<=0.0 || h>0.25)
        throw std::invalid_argument("relative perturbation must be in (0, 0.25]");
    const std::array<Profile,3> profiles{{
        {"calm",{}},
        {"current",{{0.45,-0.25,0.0},{0,0,0}}},
        {"wind",{{0,0,0},{-4.0,6.0,0.0}}},
    }};
    std::vector<SensitivityRow> rows;
    rows.reserve(profiles.size()*factors.size());
    for(const auto& profile:profiles) {
        const MarineParameters nominal{};
        const auto baseline=measure(nominal,profile.environment);
        for(const auto& factor:factors) {
            auto low=nominal, high=nominal;
            low.*(factor.field)*=(1.0-h);
            high.*(factor.field)*=(1.0+h);
            const auto lower=measure(low,profile.environment);
            const auto upper=measure(high,profile.environment);
            const double sensitivity=(upper.drift_m-lower.drift_m)/(2.0*h);
            if(!std::isfinite(sensitivity))
                throw std::runtime_error("sensitivity: nonfinite difference");
            rows.push_back({profile.name,factor.name,h,baseline.drift_m,
                            lower.drift_m,upper.drift_m,sensitivity,
                            baseline.yaw_change_rad});
        }
    }
    return rows;
}

} // namespace boat::marine
