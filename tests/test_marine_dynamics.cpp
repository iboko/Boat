#include "boat/marine_dynamics.hpp"
#include <cmath>
#include <iostream>
#include <stdexcept>

namespace {
void check(bool x,const char* s){if(!x)throw std::runtime_error(s);}
double dist(boat::ins::Vec3 v){return std::hypot(v.x,v.y);}
}

int main() {
    using namespace boat::marine;
    try {
        MarineParameters no_wave{};
        no_wave.wave_force_n=0.0;
        no_wave.wave_yaw_moment_nm=0.0;
        no_wave.wind_force_gain=0.0;
        MarineThreeDof quiet{no_wave,{}};
        MarineThreeDof river{no_wave,{{0.4,-0.25,0},{0,0,0}}};
        for(int i=0;i<1000;++i) {
            check(quiet.step(0.02,{0,0}),"neutral calm step");
            check(river.step(0.02,{0,0}),"neutral river step");
        }
        check(dist(quiet.state().position_ned_m)<1e-10,
              "calm neutral vessel moved");
        check(std::abs(river.state().position_ned_m.x-8.0)<1e-7,
              "current did not move free vessel North exactly");
        check(std::abs(river.state().position_ned_m.y+5.0)<1e-7,
              "current did not move free vessel East exactly");
        check(std::abs(river.state().velocity_ground_ned_mps.x-
                       river.state().velocity_through_water_body_mps.x-0.4)<1e-7,
              "water/ground speed confounded");
        check(!river.step(0.5,{0,0}),"time-step guard failed");
        check(!river.step(0.02,{130.0,0}),"thrust limit failed");
        check(!river.step(0.02,{10.0,60.0}),"yaw torque limit failed");

        MarineThreeDof running{no_wave,{}};
        for(int i=0;i<250;++i)
            check(running.step(0.02,{85,0}),"powered surge");
        check(running.state().position_ned_m.x>2.0,
              "thrust failed to drive physical vessel");
        check(running.state().position_ned_m.y==0,
              "zero-yaw, no-current vessel deviated laterally");
        MarineThreeDof turning{no_wave,{}};
        for(int i=0;i<200;++i)
            check(turning.step(0.02,{80,30}),"turning force balance");
        check(turning.state().heading_rad>0.25,
              "yaw moment failed to rotate hull");

        MarineThreeDof twin_straight{no_wave,{}};
        MarineThreeDof twin_starboard_turn{no_wave,{}};
        MarineThreeDof twin_port_turn{no_wave,{}};
        for(int i=0;i<200;++i) {
            check(twin_straight.stepTwinPropulsors(0.02,{40,40}),
                  "balanced twin propulsors");
            check(twin_starboard_turn.stepTwinPropulsors(0.02,{40,5}),
                  "port-heavy propulsor turn");
            check(twin_port_turn.stepTwinPropulsors(0.02,{5,40}),
                  "starboard-heavy propulsor turn");
        }
        check(std::abs(twin_straight.state().heading_rad)<1e-10,
              "matched propellers generated spurious yaw");
        check(twin_starboard_turn.state().heading_rad>0.10 &&
              twin_port_turn.state().heading_rad<-0.10,
              "twin propulsors have incorrect NED yaw signs");
        const auto before=twin_straight.state().time_s;
        check(!twin_straight.stepTwinPropulsors(0.02,{61,0}),
              "individual propulsor saturation was accepted");
        check(!twin_straight.stepTwinPropulsors(0.02,{40,NAN}),
              "nonfinite propulsor command was accepted");
        check(twin_straight.state().time_s==before,
              "invalid propulsor command mutated physical state");
        MarineParameters impossible=no_wave;
        impossible.maximum_propulsor_thrust_n=70;
        bool rejected=false;
        try { (void)MarineThreeDof{impossible,{}}; }
        catch(const std::invalid_argument&) {rejected=true;}
        check(rejected,"inconsistent propulsor/total thrust limits accepted");

        MarineParameters wave{};
        MarineThreeDof blowing{wave,{{0.4,-0.25,0},{2.5,4.0,0}}};
        for(int i=0;i<1500;++i)
            check(blowing.step(0.02,{0,0}),"wind and waves");
        check(dist(blowing.state().position_ned_m)>3.0,
              "unpowered hull with current/wind incorrectly held station");
        check(std::abs(blowing.state().position_ned_m.y-
                 river.state().position_ned_m.y)>0.1,
              "wind/wave lateral perturbation ineffective");
        std::cout<<"PASS: 3DOF surge/sway/yaw, current drift, wind/waves, "
                    "bounded port/starboard thrust and yaw signs\n";
        return 0;
    } catch(const std::exception& e) {
        std::cerr<<"FAIL: "<<e.what()<<"\n";
        return 1;
    }
}
