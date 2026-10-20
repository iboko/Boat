#include "boat/sitl_link.hpp"

#include <chrono>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>
#include <thread>

namespace {
struct Args {
    unsigned port{5762};
    unsigned duration_s{12};
    bool fixture{false};
};
Args parse(int argc,char** argv) {
    Args a{};
    for(int i=1;i<argc;++i) {
        const std::string value=argv[i];
        if(value=="--sitl-fixture") a.fixture=true;
        else if((value=="--port" || value=="--seconds") && i+1<argc) {
            const std::string n=argv[++i];
            const unsigned long number=std::stoul(n);
            if(value=="--port")a.port=static_cast<unsigned>(number);
            else a.duration_s=static_cast<unsigned>(number);
        } else throw std::invalid_argument("usage: boat_sitl_bridge --sitl-fixture [--port 5762] [--seconds 12]");
    }
    if(!a.fixture || a.port==0 || a.port>65535 || a.duration_s<2 || a.duration_s>180)
        throw std::invalid_argument("explicit --sitl-fixture required, port 1..65535, duration 2..180 seconds");
    return a;
}
}

int main(int argc,char** argv) {
    try {
        const Args a=parse(argc,argv);
        boat::sitl::TcpMavlinkLink link;
        if(!link.connectLocal(static_cast<std::uint16_t>(a.port))) {
            std::cerr<<link.error()<<"\n"; return 2;
        }
        if(!link.awaitArduPilotHeartbeat(10000)) {
            std::cerr<<link.error()<<"\n"; return 3;
        }

        // BENCHMARK FIXTURE ONLY: the position fix below is simulated.
        // This is not a radar driver or proof of GPS-independent navigation.
        boat::ins::ErrorStateIns estimator;
        estimator.initialize(0.0,{0,0,0},{0,0,0},{1,0,0,0},true);
        if(!estimator.correctPosition({0,{0,0,0},0.8,1.2,
                    boat::ins::AbsoluteSource::ShoreRadar,true,true}))return 4;
        boat::sitl::ExternalNavEvidence evidence{};
        evidence.measured_vertical_position=true;     // synthetic NED fixture
        evidence.attitude_aligned=true;               // known simulated identity
        evidence.fixed_surveyed_origin=true;          // local test origin
        evidence.shore_sensor_independent_of_gnss=true;
        evidence.gnss_denied=true;

        unsigned packets=0;
        const auto start=std::chrono::steady_clock::now();
        for(unsigned i=1;i<=a.duration_s*100;++i) {
            const double t=i*0.01;
            const boat::ins::ImuSample imu{t,{0,0,-9.80665},{0,0,0},true};
            if(!estimator.propagate(imu))return 5;
            if(i%100==0 && !estimator.correctPosition({t,{0,0,0},0.8,1.2,
                  boat::ins::AbsoluteSource::ShoreRadar,true,true}))return 6;
            if(i%10==0) {
                boat::ValidatedOdometry6D odom{};
                if(!boat::sitl::makeOdometry(estimator.state(),imu,t,evidence,odom))
                    return 7;
                if(!link.transmitOdometry(odom)) {
                    std::cerr<<link.error()<<"\n"; return 8;
                }
                ++packets;
            }
            std::this_thread::sleep_until(start+std::chrono::milliseconds(i*10));
        }
        std::cout<<"SITL_FIXTURE_ONLY target_system="<<unsigned(link.systemId())
                 <<" packets="<<packets<<" GNSS=denied"
                 <<" shore_fix=synthetic independent_position=fixture"
                 <<" outcome=MAVLINK_TRANSPORT_SENT_NOT_EKF_FUSION_PROOF\n";
        return packets>=20?0:9;
    } catch(const std::exception& e) {
        std::cerr<<"boat_sitl_bridge: "<<e.what()<<"\n";
        return 1;
    }
}
