#include "boat/marine_kinematics.hpp"
#include <cmath>
#include <iostream>
#include <stdexcept>

void check(bool ok,const char* why){if(!ok)throw std::runtime_error(why);}
int main(){
    using namespace boat::marine;
    try {
        KinematicSurfaceVessel calm;
        KinematicSurfaceVessel flowing({{0.4,-0.2,0},{0,0,0},1.8,0});
        KinematicSurfaceVessel windy({{0.4,-0.2,0},{1.5,3.0,0},1.8,0.02});
        calm.initialize(0,{0,0,0},0);
        flowing.initialize(0,{0,0,0},0);
        windy.initialize(0,{0,0,0},0);
        for(int i=0;i<200;++i) {
            check(calm.advance(0.05,1.5,0.0),"calm integration");
            check(flowing.advance(0.05,1.5,0.0),"current integration");
            check(windy.advance(0.05,1.5,0.0),"wind integration");
        }
        const auto& a=calm.state();
        const auto& b=flowing.state();
        const auto& c=windy.state();
        check(std::abs((b.position_ned_m.x-a.position_ned_m.x)-4.0)<0.01,
              "current must cause four metres North drift over ten seconds");
        check(std::abs((b.position_ned_m.y-a.position_ned_m.y)+2.0)<0.01,
              "current must cause two metres East-negative drift");
        check(std::abs(b.ground_velocity_ned_mps.x-b.water_velocity_ned_mps.x-0.4)<1e-9,
              "ground velocity must include current");
        check(std::abs(b.ground_velocity_ned_mps.y-b.water_velocity_ned_mps.y+0.2)<1e-9,
              "ground velocity E channel current");
        check(c.position_ned_m.y>b.position_ned_m.y+0.1,
              "wind-induced lateral leeway not simulated");
        check(!calm.advance(1.0,1.5,0),"refuse coarse simulation time step");
        check(!calm.advance(0.05,-3.0,0),"negative thrust rejected");
        std::cout<<"PASS: wind/leeway and current are distinct; bottom and "
                    "water reference velocities differ, 10s kinematic test\n";
        return 0;
    }catch(const std::exception& e){std::cerr<<"FAIL: "<<e.what()<<"\n";return 1;}
}
