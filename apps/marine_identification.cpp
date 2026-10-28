#include "boat/marine_identification.hpp"

#include <cmath>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {
using namespace boat::marine;

MarineControl command(double t,bool train) {
    if(train)return {28.0+15.0*std::sin(0.39*t)+8.0*std::cos(0.77*t),
                     8.0*std::sin(0.31*t)+7.0*std::cos(0.58*t)};
    return {18.0+16.0*std::cos(0.47*t),
            9.0*std::cos(0.73*t)-5.0*std::sin(0.43*t)};
}

std::vector<IdentificationSample> makeTraining(MarineParameters actual,
                                                 MarineEnvironment env) {
    MarineThreeDof vessel(actual,env);
    std::vector<IdentificationSample> data;
    data.reserve(1800);
    for(int i=0;i<1800;++i) {
        const auto before=vessel.state();
        const auto input=command(before.time_s,true);
        if(!vessel.step(0.02,input))throw std::runtime_error("training step");
        data.push_back({before,vessel.state(),input,env});
    }
    return data;
}

struct Holdout {
    double position_rmse_m{};
    double heading_rmse_rad{};
};
Holdout evaluateHoldout(MarineParameters actual,MarineParameters fitted,
                       MarineEnvironment env) {
    MarineThreeDof truth(actual,env),estimate(fitted,env);
    double squared_position=0.0,squared_heading=0.0;
    constexpr int count=600;
    for(int i=0;i<count;++i) {
        const auto input=command(truth.state().time_s,false);
        if(!truth.step(0.02,input) || !estimate.step(0.02,input))
            throw std::runtime_error("holdout step");
        const auto a=truth.state(),b=estimate.state();
        const double dn=a.position_ned_m.x-b.position_ned_m.x;
        const double de=a.position_ned_m.y-b.position_ned_m.y;
        squared_position+=dn*dn+de*de;
        squared_heading+=std::pow(a.heading_rad-b.heading_rad,2);
    }
    return {std::sqrt(squared_position/count),
            std::sqrt(squared_heading/count)};
}
}

int main(int argc,char** argv) {
    if(argc!=2 || std::string(argv[1])!="--model-only-identification") {
        std::cerr<<"Usage: boat_marine_identification --model-only-identification\n";
        return 2;
    }
    try {
        MarineParameters actual{};
        actual.mass_kg=78.0;
        actual.surge_linear_drag_ns_per_m=27.0;
        actual.yaw_inertia_kgm2=34.0;
        actual.yaw_drag_nms_per_rad=23.0;
        const MarineEnvironment env{{0.22,-0.14,0.0},{-2.0,3.0,0.0}};
        const auto identified=identifyMarineParameters(makeTraining(actual,env));
        const auto holdout=evaluateHoldout(actual,identified.parameters,env);
        std::cout<<"mass_true_kg,mass_estimated_kg,surge_drag_true_ns_per_m,"
                     "surge_drag_estimated_ns_per_m,yaw_inertia_true_kgm2,"
                     "yaw_inertia_estimated_kgm2,yaw_drag_true_nms_per_rad,"
                     "yaw_drag_estimated_nms_per_rad,surge_excitation,"
                     "yaw_excitation,surge_fit_rmse_mps2,yaw_fit_rmse_radps2,"
                     "holdout_position_rmse_m,holdout_heading_rmse_rad\n";
        std::cout<<std::fixed<<std::setprecision(10)
                 <<actual.mass_kg<<','<<identified.parameters.mass_kg<<','
                 <<actual.surge_linear_drag_ns_per_m<<','
                 <<identified.parameters.surge_linear_drag_ns_per_m<<','
                 <<actual.yaw_inertia_kgm2<<','
                 <<identified.parameters.yaw_inertia_kgm2<<','
                 <<actual.yaw_drag_nms_per_rad<<','
                 <<identified.parameters.yaw_drag_nms_per_rad<<','
                 <<identified.surge_excitation<<','<<identified.yaw_excitation<<','
                 <<identified.surge_residual_rmse_mps2<<','
                 <<identified.yaw_residual_rmse_radps2<<','
                 <<holdout.position_rmse_m<<','<<holdout.heading_rmse_rad<<'\n';
        return 0;
    } catch(const std::exception& e) {
        std::cerr<<"FAIL_MARINE_IDENTIFICATION: "<<e.what()<<'\n';
        return 1;
    }
}
