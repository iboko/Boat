#include "boat/sensor_fixture.hpp"
#include <cmath>
#include <iostream>
#include <stdexcept>

namespace {
void require(bool ok,const char* why) {
    if(!ok)throw std::runtime_error(why);
}
}

int main() {
    using namespace boat::fixture;
    using boat::ins::Vec3;
    try {
        SensorFusion f{};
        for(int i=0;i<=240;++i) {
            const double t=i*0.1;
            const PhysicsFrame physical{t,{t,0,0},{1,0,0},{1,0,0,0},{0,0,0}};
            require(f.step(physical),"sensor and ESKF propagation");
        }
        require(f.propagated()>=500,"real ESKF not propagated");
        require(f.generator().shoreCount()>=20,"shore radar fixes not simulated");
        require(f.generator().dvlCount()>=50,"bottom lock Doppler fixes not simulated");
        require(f.generator().missedShoreCount()>0,"radar outage not exercised");
        require(f.generator().missedDvlCount()>0,"bottom-track outage not exercised");
        require(f.state().last_gnss_fix_s<0.0,"GNSS mistakenly fused");
        const auto estimated=f.state().position_ned_m;
        require(std::hypot(estimated.x-24.0,estimated.y)<1.5,
                "ESKF failed straight-line position accuracy in simple fixture");
        require(f.integrity(),"integrity should recover after radar outage");

        // Fail-safe: with no independent absolute position for >30 seconds,
        // DVL velocity updates cannot substitute for the missing shore fix.
        Scenario outage{};
        outage.radar_dropout_from_s=1.0;
        outage.radar_dropout_until_s=90.0;
        SensorFusion denied(outage);
        for(int i=0;i<=450;++i) {
            const double t=i*0.1;
            require(denied.step({t,{0,0,0},{0,0,0},{1,0,0,0},{0,0,0}}),
                    "outage ESKF propagation");
        }
        require(!denied.integrity(),"stale shore fixes must block navigation");
        require(denied.state().last_gnss_fix_s<0.0,
                "GNSS must not reappear in denial test");

        // Raw water-relative speed must not masquerade as bottom-track DVL.
        boat::ins::ErrorStateIns ins;
        ins.initialize(0,{0,0,0},{0,0,0},{1,0,0,0},true);
        require(!ins.correctGroundVelocity({0,{0,0,0},0.1,0.1,false,true}),
                "accepting water-relative log speed is unsafe");
        std::cout<<"PASS: 3D simulated IMU, radar and DVL -> ESKF, "
                    "dropouts, no-GNSS and stale-fix stop\n";
        return 0;
    } catch(const std::exception& e) {
        std::cerr<<"FAIL: "<<e.what()<<"\n";
        return 1;
    }
}
