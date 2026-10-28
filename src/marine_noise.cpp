#include "boat/marine_noise.hpp"
#include "boat/marine_windowed.hpp"

#include <algorithm>
#include <cmath>
#include <random>
#include <stdexcept>
#include <vector>

namespace boat::marine {
namespace {

MarineParameters actualParameters() {
    MarineParameters p{};
    p.mass_kg=78.0;
    p.surge_linear_drag_ns_per_m=27.0;
    p.yaw_inertia_kgm2=34.0;
    p.yaw_drag_nms_per_rad=23.0;
    return p;
}

std::vector<IdentificationSample> generateRecord(MarineParameters actual) {
    const MarineEnvironment env{{0.22,-0.14,0.0},{-2.0,3.0,0.0}};
    MarineThreeDof vessel(actual,env);
    std::vector<IdentificationSample> clean;
    clean.reserve(1800);
    for(int i=0;i<1800;++i) {
        const auto before=vessel.state();
        const double t=before.time_s;
        const MarineControl command{
            28.0+15.0*std::sin(0.39*t)+8.0*std::cos(0.77*t),
            8.0*std::sin(0.31*t)+7.0*std::cos(0.58*t)};
        if(!vessel.step(0.02,command))
            throw std::runtime_error("noise experiment: forward step failed");
        clean.push_back({before,vessel.state(),command,env});
    }
    return clean;
}

double percentile(const std::vector<double>& sorted,double proportion) {
    const double index=proportion*static_cast<double>(sorted.size()-1);
    const auto low=static_cast<std::size_t>(std::floor(index));
    const auto high=static_cast<std::size_t>(std::ceil(index));
    return sorted[low]+(sorted[high]-sorted[low])*(index-low);
}

} // namespace

NoiseTrialSummary evaluateMarineNoise(NoiseTrialSettings settings,
                                     std::size_t window_samples) {
    const bool valid=std::isfinite(settings.speed_std_mps) &&
        std::isfinite(settings.yaw_rate_std_radps) &&
        std::isfinite(settings.heading_std_rad) &&
        settings.speed_std_mps>=0.0 && settings.speed_std_mps<=0.05 &&
        settings.yaw_rate_std_radps>=0.0 &&
        settings.yaw_rate_std_radps<=0.02 &&
        settings.heading_std_rad>=0.0 && settings.heading_std_rad<=0.05 &&
        settings.trials>=5 && settings.trials<=500 &&
        (window_samples==1 || (window_samples>=2 && window_samples<=200));
    if(!valid)throw std::invalid_argument("invalid marine noise experiment setup");
    const auto actual=actualParameters();
    const auto clean=generateRecord(actual);
    std::mt19937_64 engine(settings.seed);
    std::normal_distribution<double> normal(0.0,1.0);
    std::array<std::vector<double>,4> samples;
    std::size_t accepted=0;
    for(std::size_t trial=0;trial<settings.trials;++trial) {
        std::vector<MarineState> states;
        states.reserve(clean.size()+1);
        states.push_back(clean.front().before);
        for(const auto& row:clean)states.push_back(row.after);
        for(auto& state:states) {
            // One random draw per sensor per timestamp. Adjacent samples share
            // the same noisy state, as real finite-difference logs do.
            state.velocity_through_water_body_mps.x+=
                settings.speed_std_mps*normal(engine);
            state.velocity_through_water_body_mps.y+=
                settings.speed_std_mps*normal(engine);
            state.yaw_rate_radps+=settings.yaw_rate_std_radps*normal(engine);
            state.heading_rad+=settings.heading_std_rad*normal(engine);
        }
        std::vector<IdentificationSample> noisy;
        noisy.reserve(clean.size());
        for(std::size_t i=0;i<clean.size();++i)
            noisy.push_back({states[i],states[i+1],clean[i].applied,
                             clean[i].environment});
        try {
            const auto parameters=window_samples==1 ?
                identifyMarineParameters(noisy).parameters :
                identifyMarineParametersWindowed(noisy,window_samples).parameters;
            const std::array<double,4> values{{
                parameters.mass_kg,
                parameters.surge_linear_drag_ns_per_m,
                parameters.yaw_inertia_kgm2,
                parameters.yaw_drag_nms_per_rad}};
            for(std::size_t i=0;i<values.size();++i)samples[i].push_back(values[i]);
            ++accepted;
        } catch(const std::invalid_argument&) {
            // Rejected estimates are counted, never replaced with nominal values.
        }
    }
    NoiseTrialSummary summary{};
    summary.trials=settings.trials;
    summary.accepted=accepted;
    summary.rejected=settings.trials-accepted;
    const std::array<double,4> truths{{actual.mass_kg,
        actual.surge_linear_drag_ns_per_m,actual.yaw_inertia_kgm2,
        actual.yaw_drag_nms_per_rad}};
    for(std::size_t i=0;i<truths.size();++i) {
        auto& current=samples[i];
        if(current.empty()) {
            summary.estimates[i]={truths[i]};
            continue;
        }
        std::sort(current.begin(),current.end());
        double sum=0.0;
        for(double value:current)sum+=value;
        const double mean=sum/static_cast<double>(current.size());
        summary.estimates[i]={truths[i],mean,mean/truths[i]-1.0,
                              percentile(current,0.05),percentile(current,0.95)};
    }
    return summary;
}

} // namespace boat::marine
