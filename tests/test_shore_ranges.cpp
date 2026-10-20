#include "boat/shore_ranges.hpp"
#include "boat/sensor_fixture.hpp"
#include <cmath>
#include <iostream>
#include <stdexcept>

namespace {
void require(bool value,const char* text){
    if(!value)throw std::runtime_error(text);
}
}

int main(){
    using namespace boat::fixture;
    using boat::ins::Vec3;
    try {
        // No absolute position passed to solver, only five radar ranges
        // associated with fixed surveyed landmarks.
        for(double time:{0.0,1.0,3.5,4.2,6.0}) {
            const Vec3 truth{10.0+0.8*time,-3.0+time*0.4,0.0};
            const ShoreRanges obs=ShoreRangeModel::measure(truth,time);
            require(obs.valid && obs.count==5,"expected five landmarks in range");
            const ShoreSolution solved=ShoreRangeModel::solve(obs,{0,0,0});
            require(solved.valid,"range solution failed");
            require(solved.inliers>=4,"landmark outlier should not block solution");
            require(std::hypot(solved.position_ned_m.x-truth.x,
                               solved.position_ned_m.y-truth.y)<0.8,
                    "multilateration diverged");
            if(time>=3.0 && time<5.0)
                require(solved.inliers==4,"must reject injected 12m outlier");
        }
        auto duplicates=ShoreRangeModel::measure({0,0,0},2.0);
        duplicates.readings[1].reflector_id=
            duplicates.readings[0].reflector_id;
        require(!ShoreRangeModel::solve(duplicates,{0,0,0}).valid,
                "duplicate reflector ID must not inflate inlier count");
        auto insufficient=ShoreRangeModel::measure({0,0,0},0.0);
        insufficient.count=2;
        require(!ShoreRangeModel::solve(insufficient,{0,0,0}).valid,
                "unobservable two-range geometry must fail");
        auto bad=ShoreRangeModel::measure({0,0,0},0.0);
        bad.readings[0].distance_m=-2;
        require(!ShoreRangeModel::solve(bad,{0,0,0}).valid,
                "invalid ranges must be rejected");

        Scenario scenario{};
        SyntheticSensors generator(scenario);
        bool checked_water=false;
        for(int i=0;i<=150;++i) {
            const double t=i*0.1;
            const auto obs=generator.observe({t,{0,0,0},{1,0,0},
                                                 {1,0,0,0},{0,0,0}});
            require(obs.valid,"synthetic IMU");
            if(obs.water_track_available) {
                require(!obs.water_track.ground_referenced,
                        "water-track DVL must not claim ground velocity");
                require(!obs.bottom_track_available,
                        "bottom and water tracking cannot coexist");
                // Filter operates only in ground frame; water-relative
                // motion is reduced by a nonzero 0.35 m/s current.
                if(std::abs(t-11.0)<0.11) {
                    require(obs.water_track.velocity_ned_mps.x<0.8,
                            "nonzero modeled water current not applied");
                    checked_water=true;
                }
            }
        }
        require(checked_water && generator.waterModeCount()>0,
                "water-track mode under bottom-loss must be exercised");
        require(generator.radarOutlierCount()>0,
                "landmark outlier must be encountered");
        require(generator.missedShoreCount()>0,
                "outage model must include shore-radar dropout");
        std::cout<<"PASS: 5 reflector range-only 2D fix, 12m outlier, "
                    "ill-posed range rejection, water-track vs bottom-lock DVL\n";
    } catch(const std::exception& e) {
        std::cerr<<"FAIL: "<<e.what()<<"\n";
        return 1;
    }
}
