#include "boat/marine_noise.hpp"

#include <iomanip>
#include <iostream>
#include <string>

int main(int argc,char** argv) {
    if(argc!=2 || std::string(argv[1])!="--model-only-noise") {
        std::cerr<<"Usage: boat_marine_noise --model-only-noise\n";
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
        std::cout<<"profile,parameter,speed_sigma_mps,yaw_rate_sigma_radps,"
                     "heading_sigma_rad,trials,accepted,rejected,truth,mean,"
                     "relative_bias,p05,p95\n";
        std::cout<<std::fixed<<std::setprecision(9);
        for(const auto& profile:profiles) {
            const auto result=evaluateMarineNoise(profile.setting);
            for(std::size_t i=0;i<result.estimates.size();++i) {
                const auto& value=result.estimates[i];
                std::cout<<profile.name<<','<<names[i]<<','
                         <<profile.setting.speed_std_mps<<','
                         <<profile.setting.yaw_rate_std_radps<<','
                         <<profile.setting.heading_std_rad<<','
                         <<result.trials<<','<<result.accepted<<','
                         <<result.rejected<<','<<value.truth<<','
                         <<value.mean<<','<<value.relative_bias<<','
                         <<value.p05<<','<<value.p95<<'\n';
            }
        }
        return 0;
    } catch(const std::exception& error) {
        std::cerr<<"FAIL_MARINE_NOISE: "<<error.what()<<'\n';
        return 1;
    }
}
