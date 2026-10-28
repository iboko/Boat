#include "boat/marine_noise.hpp"

#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>

namespace {
void check(bool ok,const char* why) {
    if(!ok)throw std::runtime_error(why);
}
}

int main() {
    using namespace boat::marine;
    try {
        const auto ideal=evaluateMarineNoise({0.0,0.0,0.0,10,42});
        check(ideal.accepted==10 && ideal.rejected==0,
              "noiseless identification unexpectedly failed");
        for(const auto& e:ideal.estimates)
            check(std::abs(e.relative_bias)<1e-7 &&
                  std::abs(e.mean-e.truth)<1e-5 &&
                  std::abs(e.p95-e.p05)<1e-7,
                  "noiseless simulation is not an exact control");

        const NoiseTrialSettings setup{0.003,0.001,0.002,40,7819};
        const auto noisy=evaluateMarineNoise(setup);
        const auto repeated=evaluateMarineNoise(setup);
        check(noisy.trials==40 && noisy.accepted>0 &&
              noisy.accepted+noisy.rejected==noisy.trials,
              "noise trial accounting failed");
        bool different=false;
        for(std::size_t i=0;i<noisy.estimates.size();++i) {
            const auto& e=noisy.estimates[i];
            const auto& r=repeated.estimates[i];
            check(std::isfinite(e.mean) && std::isfinite(e.relative_bias) &&
                  std::isfinite(e.p05) && std::isfinite(e.p95) &&
                  e.p05<=e.p95,
                  "nonfinite or inverted noisy percentile interval");
            check(e.mean==r.mean && e.p05==r.p05 && e.p95==r.p95,
                  "noise experiment is not seed-reproducible");
            if(e.p95-e.p05>1e-3)different=true;
        }
        check(different,"measurement noise produced no estimate dispersion");

        bool rejected=false;
        try {(void)evaluateMarineNoise({-0.01,0.0,0.0,40,42});}
        catch(const std::invalid_argument&){rejected=true;}
        check(rejected,"negative sensor uncertainty accepted");
        rejected=false;
        try {(void)evaluateMarineNoise({0.001,0.0,0.0,2,42});}
        catch(const std::invalid_argument&){rejected=true;}
        check(rejected,"statistically meaningless trial count accepted");
        rejected=false;
        try {(void)evaluateMarineNoise({0.001,std::numeric_limits<double>::quiet_NaN(),
                                        0.0,40,42});}
        catch(const std::invalid_argument&){rejected=true;}
        check(rejected,"NaN sensor noise accepted");
        std::cout<<"PASS: marine noise Monte Carlo with empirical intervals, "
                    "seed reproducibility, and rejection accounting\n";
        return 0;
    } catch(const std::exception& error) {
        std::cerr<<"FAIL: "<<error.what()<<'\n';
        return 1;
    }
}
