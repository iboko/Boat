#include "boat/marine_windowed.hpp"
#include "boat/marine_noise.hpp"

#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <vector>

namespace {
using namespace boat::marine;

void check(bool ok,const char* message) {
    if(!ok)throw std::runtime_error(message);
}

std::vector<IdentificationSample> makeRecord(MarineParameters truth) {
    const MarineEnvironment environment{{0.2,-0.1,0.0},{-2.0,3.0,0.0}};
    MarineThreeDof vessel(truth,environment);
    std::vector<IdentificationSample> samples;
    samples.reserve(1200);
    for(int i=0;i<1200;++i) {
        const auto before=vessel.state();
        const double t=before.time_s;
        const MarineControl control{30.0+15.0*std::sin(0.37*t),
                                    8.0*std::cos(0.53*t)+4.0*std::sin(0.21*t)};
        check(vessel.step(0.02,control),"failed to generate clean records");
        samples.push_back({before,vessel.state(),control,environment});
    }
    return samples;
}

double error(double estimate,double truth) {
    return std::abs(estimate/truth-1.0);
}

} // namespace

int main() {
    using namespace boat::marine;
    try {
        MarineParameters truth{};
        truth.mass_kg=74.0;
        truth.surge_linear_drag_ns_per_m=31.0;
        truth.yaw_inertia_kgm2=33.0;
        truth.yaw_drag_nms_per_rad=24.0;
        const auto clean=makeRecord(truth);
        const auto fit=identifyMarineParametersWindowed(clean,25);
        check(fit.used_samples==1200 && fit.windows==48,
              "incorrect number of integration windows");
        check(fit.surge_excitation>1e-4 && fit.yaw_excitation>1e-4,
              "missing excitation in test maneuver");
        check(error(fit.parameters.mass_kg,truth.mass_kg)<1e-8 &&
              error(fit.parameters.surge_linear_drag_ns_per_m,
                    truth.surge_linear_drag_ns_per_m)<1e-8 &&
              error(fit.parameters.yaw_inertia_kgm2,
                    truth.yaw_inertia_kgm2)<1e-8 &&
              error(fit.parameters.yaw_drag_nms_per_rad,
                    truth.yaw_drag_nms_per_rad)<1e-8,
              "integrated equations disagree with the forward model");
        check(fit.surge_integral_rmse_mps<1e-6 &&
              fit.yaw_integral_rmse_radps<1e-6,
              "unexpected clean integral residual");

        // The comparison uses identical seeded measurement-noise sequences.
        const NoiseTrialSettings moderate{0.005,0.002,0.005,60,20261028};
        const auto instant=evaluateMarineNoise(moderate);
        const auto windowed=evaluateMarineNoise(moderate,25);
        check(instant.accepted==60 && windowed.accepted==60,
              "moderate-noise regression does not have equal sample counts");
        for(std::size_t i=0;i<4;++i) {
            const double a=std::abs(instant.estimates[i].relative_bias);
            const double b=std::abs(windowed.estimates[i].relative_bias);
            check(b<a,"integral method failed to reduce observed mean bias");
        }
        check(std::abs(windowed.estimates[0].relative_bias)<0.005 &&
              std::abs(instant.estimates[0].relative_bias)>0.03,
              "mass bias reduction is not reproduced");

        bool rejected=false;
        try {(void)identifyMarineParametersWindowed(clean,1);}
        catch(const std::invalid_argument&){rejected=true;}
        check(rejected,"one-sample integration window accepted");
        rejected=false;
        try {(void)identifyMarineParametersWindowed(clean,201);}
        catch(const std::invalid_argument&){rejected=true;}
        check(rejected,"unbounded integration window accepted");

        auto corrupted=clean;
        corrupted[10].before.velocity_through_water_body_mps.x+=0.2;
        rejected=false;
        try {(void)identifyMarineParametersWindowed(corrupted,25);}
        catch(const std::invalid_argument&){rejected=true;}
        check(rejected,"discontinuous sample boundaries accepted");

        corrupted=clean;
        corrupted[0].after.yaw_rate_radps=
            std::numeric_limits<double>::quiet_NaN();
        rejected=false;
        try {(void)identifyMarineParametersWindowed(corrupted,25);}
        catch(const std::invalid_argument&){rejected=true;}
        check(rejected,"nonfinite sample accepted");

        std::vector<IdentificationSample> empty_motion(100);
        for(std::size_t i=0;i<empty_motion.size();++i) {
            empty_motion[i].before.time_s=i*0.02;
            empty_motion[i].after.time_s=(i+1)*0.02;
        }
        rejected=false;
        try {(void)identifyMarineParametersWindowed(empty_motion,25);}
        catch(const std::invalid_argument&){rejected=true;}
        check(rejected,"unidentifiable neutral-motion data accepted");

        rejected=false;
        try {(void)evaluateMarineNoise(moderate,201);}
        catch(const std::invalid_argument&){rejected=true;}
        check(rejected,"invalid Monte Carlo window size accepted");
        std::cout<<"PASS: windowed integration exactly matches clean model, "
                    "reduces seeded noise bias, rejects weak/bad data\n";
        return 0;
    } catch(const std::exception& e) {
        std::cerr<<"FAIL: "<<e.what()<<'\n';
        return 1;
    }
}
