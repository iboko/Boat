#include "boat/sensor_fixture.hpp"
#include <cmath>
#include <algorithm>
#include <iostream>
#include <stdexcept>
static void demand(bool condition,const char* why){
    if(!condition)throw std::runtime_error(why);
}
int main(){
    using namespace boat::fixture;
    using boat::ins::Vec3;
    using boat::ins::Quaternion;
    using boat::ins::expRotation;
    using boat::ins::multiply;
    try {
        SensorFusion filter;
        double max_angle_error=0.0;
        for(int i=0;i<=300;++i) {
            const double time=i*0.10;
            const double roll=0.17*std::sin(1.9*time);
            const double pitch=0.13*std::sin(1.4*time+0.4);
            const Quaternion actual=multiply(expRotation({0,pitch,0}),
                                              expRotation({roll,0,0}));
            // Deliberately provide wrongly zero angular-rate channel.
            // SITL fixture must compute FRD gyroscope from consistent
            // consecutive quaternions, not trust a mismatched gyro basis.
            demand(filter.step({time,{0,0,0},{0,0,0},actual,{0,0,0}}),
                   "wave-driven synthetic ESKF step failed");
            const auto reference=boat::ins::rotate(actual,{0,0,1});
            const auto estimated=boat::ins::rotate(filter.state().body_to_ned,
                                                    {0,0,1});
            const double angle=boat::ins::norm(reference-estimated);
            if(time>1.0)max_angle_error=std::max(max_angle_error,angle);
        }
        demand(max_angle_error<0.075,
               "attitude under 10-degree wave motions not tracked");
        demand(filter.acceptedAttitudes()>270,
               "wave attitude fixes were gate-rejected");
        demand(filter.rejectedAttitudes()<10,
               "systematic gyro frame disagreement persists");
        const auto p=filter.state().position_ned_m;
        demand(std::hypot(p.x,p.y)<0.9,"wave-like rocking led to INS drift");
        demand(filter.integrity(),"nominal wave fixture should be navigable");
        std::cout<<"PASS: 3D wave-driven virtual gyro from quaternion increments;"
                    " attitude ESKF aiding stays within 0.075 rad\n";
        return 0;
    }catch(const std::exception& e) {
        std::cerr<<"FAIL: "<<e.what()<<"\n";
        return 1;
    }
}
