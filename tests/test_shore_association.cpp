#include "boat/shore_association.hpp"
#include "boat/sensor_fixture.hpp"
#include <cmath>
#include <iostream>
#include <stdexcept>
void demand(bool condition,const char* why){
    if(!condition)throw std::runtime_error(why);
}
int main(){
    using namespace boat::fixture;
    try{
        for(double t:{0.0,3.5,5.6,9.7,21.0}){
            const boat::ins::Vec3 truth{0.5*t,0.3*t,0};
            const auto echoes=ShoreMapAssociator::simulateEchoes(truth,t);
            demand(echoes.valid && echoes.count==5,"five anonymous echoes missing");
            // Unlabelled measurements must require an independent,
            // trustworthy approximate pose prior.
            const auto association=ShoreMapAssociator::associate(
                echoes,{truth.x+0.16,truth.y-0.13,0});
            demand(association.valid,"map association rejected good pose prior");
            demand(association.matched>=4,"four distinct landmarks needed");
            const auto fixed=ShoreRangeModel::solve(
                association.ranges,{truth.x+0.16,truth.y-0.13,0});
            demand(fixed.valid,"no valid position from associated returns");
            demand(std::hypot(fixed.position_ned_m.x-truth.x,
                              fixed.position_ned_m.y-truth.y)<0.6,
                   "map association induced wrong position");
            if(t==3.5)
                demand(association.unmatched>=1,
                       "12 metre outlier must be discarded before fusion");
        }
        auto echoes=ShoreMapAssociator::simulateEchoes({0,0,0},3.5);
        demand(!ShoreMapAssociator::associate(echoes,{250,250,0}).valid,
               "untrusted far pose must not allow map assignment");
        auto bad=echoes;
        bad.ranges_m[0]=-42.0;
        demand(!ShoreMapAssociator::associate(bad,{0,0,0}).valid,
               "invalid radar echo accepted");

        Scenario lost{};
        lost.radar_dropout_from_s=6;
        lost.radar_dropout_until_s=200;
        lost.max_non_gnss_fix_age_s=3;
        SensorFusion fusion(lost);
        bool observed_stop=false;
        for(int i=0;i<=160;++i) {
            const double now=i*0.1;
            demand(fusion.step({now,{0,0,0},{0,0,0},
                   {1,0,0,0},{0,0,0}}),"failed IMU propagation");
            if(now<8) demand(fusion.integrity(),
                            "premature safety stop");
            if(now>10 && !fusion.integrity())observed_stop=true;
        }
        demand(observed_stop,"prolonged loss of shoreline fails to stop navigation");
        demand(fusion.state().last_gnss_fix_s<0,
               "GNSS must remain disabled");
        // Gazebo may require a long firmware-clock pre-arm warmup.
        // Radar failure must not start until the supervisor has received
        // ARMED+AUTO, rather than counting from when the bridge boots.
        Scenario defer{};
        defer.radar_dropout_from_s=600.0;
        defer.radar_dropout_until_s=600.0;
        defer.max_non_gnss_fix_age_s=3.0;
        SensorFusion on_arm(defer);
        bool fired=false;
        for(int i=0;i<=250;++i) {
            const double t=i*0.1;
            demand(on_arm.step({t,{0,0,0},{0,0,0},
                   {1,0,0,0},{0,0,0}}),"delayed radar test propagation");
            if(i==100) {
                demand(on_arm.beginRadarBlackoutAfterArm(),
                       "supervisor could not trigger armed radar outage");
                fired=true;
            }
            if(t<15.5) {
                demand(on_arm.generator().missedShoreCount()==0,
                       "radar blackout began before the armed supervisor delay");
                demand(on_arm.integrity(),
                       "navigation stale during pre-arm warmup");
            }
        }
        demand(fired && on_arm.generator().missedShoreCount()>0,
               "armed synthetic radar outage never activated");
        demand(!on_arm.integrity(),
               "continued radar blackout did not trigger HOLD integrity gate");
        std::cout<<"PASS: shuffled anonymous radar echoes, false return gated, "
                    "position prior required, stale shore position triggers HOLD request\n";
        return 0;
    } catch (const std::exception& e) {
        std::cerr<<"FAIL: "<<e.what()<<"\n";return 1;
    }
}
