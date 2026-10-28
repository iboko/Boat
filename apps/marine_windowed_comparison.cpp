#include "boat/marine_noise.hpp"

#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>

int main(int argc,char** argv) {
    if(argc!=2 || std::string(argv[1])!="--model-only-compare") {
        std::cerr<<"Usage: boat_marine_windowed_comparison --model-only-compare\n";
        return 2;
    }
    using namespace boat::marine;
    try {
        struct Profile {const char* name; NoiseTrialSettings setting;};
        const Profile profiles[]{{"ideal",{0.0,0.0,0.0,60,20261028}},
                                 {"low",{0.001,0.0005,0.001,60,20261028}},
                                 {"moderate",{0.005,0.002,0.005,60,20261028}}};
        const char* names[]{"mass_kg","surge_linear_drag_ns_per_m",
                            "yaw_inertia_kgm2","yaw_drag_nms_per_rad"};
        constexpr std::size_t window=25; // 25 x 0.02 s = 0.5 s.
        std::cout<<"profile,parameter,window_samples,trials,instant_accepted,"
                     "windowed_accepted,truth,instant_mean,windowed_mean,"
                     "instant_relative_bias,windowed_relative_bias,"
                     "windowed_p05,windowed_p95\n";
        std::cout<<std::fixed<<std::setprecision(9);
        for(const auto& profile:profiles) {
            const auto instant=evaluateMarineNoise(profile.setting);
            const auto integrated=evaluateMarineNoise(profile.setting,window);
            for(std::size_t i=0;i<instant.estimates.size();++i) {
                const auto& a=instant.estimates[i];
                const auto& b=integrated.estimates[i];
                std::cout<<profile.name<<','<<names[i]<<','<<window<<','
                         <<profile.setting.trials<<','<<instant.accepted<<','
                         <<integrated.accepted<<','<<a.truth<<','
                         <<a.mean<<','<<b.mean<<','<<a.relative_bias<<','
                         <<b.relative_bias<<','<<b.p05<<','<<b.p95<<'\n';
            }
        }
        return 0;
    } catch(const std::exception& e) {
        std::cerr<<"FAIL_MARINE_WINDOWED_COMPARISON: "<<e.what()<<'\n';
        return 1;
    }
}
