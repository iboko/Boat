#include "boat/marine_identification.hpp"

#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <vector>

namespace {
using namespace boat::marine;

void check(bool ok,const char* reason) {
    if(!ok)throw std::runtime_error(reason);
}

MarineControl input(double t,int profile) {
    if(profile==0)
        return {28.0+15.0*std::sin(0.39*t)+8.0*std::cos(0.77*t),
                8.0*std::sin(0.31*t)+7.0*std::cos(0.58*t)};
    return {18.0+16.0*std::cos(0.47*t),
            9.0*std::cos(0.73*t)-5.0*std::sin(0.43*t)};
}

std::vector<IdentificationSample> sample(MarineParameters true_values,
                                         MarineEnvironment environment,
                                         int profile,int steps) {
    MarineThreeDof model(true_values,environment);
    std::vector<IdentificationSample> series;
    series.reserve(static_cast<std::size_t>(steps));
    for(int i=0;i<steps;++i) {
        const auto before=model.state();
        const auto command=input(before.time_s,profile);
        check(model.step(0.02,command),"failed to generate synthetic record");
        series.push_back({before,model.state(),command,environment});
    }
    return series;
}
double absRelative(double estimate,double truth) {
    return std::abs(estimate/truth-1.0);
}
}

int main() {
    using namespace boat::marine;
    try {
        MarineParameters truth{};
        truth.mass_kg=78.0;
        truth.surge_linear_drag_ns_per_m=27.0;
        truth.yaw_inertia_kgm2=34.0;
        truth.yaw_drag_nms_per_rad=23.0;
        MarineEnvironment environmental{{0.22,-0.14,0.0},{-2.0,3.0,0.0}};
        const auto train=sample(truth,environmental,0,1800);
        const auto fit=identifyMarineParameters(train);
        check(fit.sample_count==1800,"wrong sample count");
        check(fit.surge_excitation>1e-3 && fit.yaw_excitation>1e-3,
              "test did not independently excite both regressions");
        check(absRelative(fit.parameters.mass_kg,truth.mass_kg)<1e-8,
              "mass identification failure");
        check(absRelative(fit.parameters.surge_linear_drag_ns_per_m,
                          truth.surge_linear_drag_ns_per_m)<1e-8,
              "surge drag identification failure");
        check(absRelative(fit.parameters.yaw_inertia_kgm2,
                          truth.yaw_inertia_kgm2)<1e-8,
              "yaw inertia identification failure");
        check(absRelative(fit.parameters.yaw_drag_nms_per_rad,
                          truth.yaw_drag_nms_per_rad)<1e-8,
              "yaw drag identification failure");
        check(fit.surge_residual_rmse_mps2<1e-6 &&
              fit.yaw_residual_rmse_radps2<1e-6,
              "noiseless training data has excessive residual");

        // The second excitation is deliberately not used for fitting.
        MarineThreeDof expected(truth,environmental);
        MarineThreeDof predicted(fit.parameters,environmental);
        for(int i=0;i<600;++i) {
            const auto cmd=input(expected.state().time_s,1);
            check(expected.step(0.02,cmd) && predicted.step(0.02,cmd),
                  "holdout simulation step failed");
        }
        const auto a=expected.state(),b=predicted.state();
        check(std::hypot(a.position_ned_m.x-b.position_ned_m.x,
                         a.position_ned_m.y-b.position_ned_m.y)<1e-7 &&
              std::abs(a.heading_rad-b.heading_rad)<1e-7,
              "unseen control program was not reproduced");

        bool rejected=false;
        try {(void)identifyMarineParameters({});}
        catch(const std::invalid_argument&){rejected=true;}
        check(rejected,"empty identification passed");

        std::vector<IdentificationSample> degenerate(12);
        for(std::size_t i=0;i<degenerate.size();++i) {
            degenerate[i].before.time_s=0.02*static_cast<double>(i);
            degenerate[i].after.time_s=degenerate[i].before.time_s+0.02;
        }
        rejected=false;
        try {(void)identifyMarineParameters(degenerate);}
        catch(const std::invalid_argument&){rejected=true;}
        check(rejected,"unexcited coefficients falsely identified");

        auto corrupted=train;
        corrupted[0].after.time_s=corrupted[0].before.time_s;
        rejected=false;
        try {(void)identifyMarineParameters(corrupted);}
        catch(const std::invalid_argument&){rejected=true;}
        check(rejected,"zero-duration observation accepted");

        corrupted=train;
        corrupted[8].before.velocity_through_water_body_mps.x=
            std::numeric_limits<double>::quiet_NaN();
        rejected=false;
        try {(void)identifyMarineParameters(corrupted);}
        catch(const std::invalid_argument&){rejected=true;}
        check(rejected,"NaN observation accepted");

        corrupted=train;
        corrupted[4].environment.wind_ned_mps.z=0.5;
        rejected=false;
        try {(void)identifyMarineParameters(corrupted);}
        catch(const std::invalid_argument&){rejected=true;}
        check(rejected,"3D wind input accepted by planar estimator");

        // A deliberately inconsistent physical-force log must not appear exact.
        corrupted=train;
        for(auto& obs:corrupted)obs.applied.forward_thrust_n*=0.8;
        const auto biased=identifyMarineParameters(corrupted);
        check(absRelative(biased.parameters.mass_kg,truth.mass_kg)>0.05,
              "force miscalibration was not reflected in mass estimate");

        std::cout<<"PASS: four-coefficient marine identification, independent "
                    "holdout, rank and log-validation guards\n";
        return 0;
    } catch(const std::exception& e) {
        std::cerr<<"FAIL: "<<e.what()<<'\n';
        return 1;
    }
}
