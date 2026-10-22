#include "boat/ins.hpp"
#include <cmath>
#include <iostream>
#include <stdexcept>

static void ensure(bool cond,const char* why){
    if(!cond)throw std::runtime_error(why);
}
int main() {
    using namespace boat::ins;
    try {
        ErrorStateIns filter;
        filter.initialize(0.0,{0,0,0},{0,0,0},{1,0,0,0},true);
        unsigned fused=0;
        for(int i=1;i<=2000;++i) {
            const double t=i*0.01;
            // Test IMU carries fixed uncalibrated gyro bias 0.03 rad/s.
            // Independent synthetic inclinometer/compass reports level.
            ensure(filter.propagate({t,{0,0,-9.80665},{0.03,0,0},true}),
                   "failed IMU propagation");
            if(i%10==0) {
                const AbsoluteAttitude measured{
                    t,normalize(expRotation({0.002*std::sin(t),0,0})),
                    0.025,true,true};
                if(filter.correctAttitude(measured))++fused;
            }
        }
        ensure(fused>=170,"independent attitude updates rejected too often");
        const auto tilt=rotate(filter.state().body_to_ned,{0,0,1});
        ensure(std::hypot(tilt.x,tilt.y)<0.035,
               "ESKF attitude diverged despite 10Hz external tilt aiding");
        ensure(norm(filter.state().position_ned_m)<2.0,
               "uncompensated gravity under small gyro drift caused navigation explosion");
        ensure(!filter.correctAttitude({
            filter.state().time_s,expRotation({2.9,0,0}),
            0.025,true,true}),"gross attitude outlier accepted");
        ensure(!filter.correctAttitude({
            filter.state().time_s,Quaternion{},
            0.025,false,true}),"untrusted attitude marked independent");
        std::cout<<"PASS: ESKF attitude aiding rejects false compass and"
                    " corrects gyro bias in GNSS-denied gravity mechanization\n";
        return 0;
    } catch(const std::exception& exc) {
        std::cerr<<"FAIL: "<<exc.what()<<"\n";
        return 1;
    }
}
