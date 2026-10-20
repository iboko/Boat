#include "boat/sitl_link.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <limits>
#include <poll.h>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {
constexpr double kHomeLat=-35.363261;
constexpr double kHomeLon=149.165230;
constexpr double kHomeAltM=584.0;
constexpr double kRad=3.14159265358979323846/180.0;
constexpr double kLatScale=111132.0;
const double kLonScale=111320.0*std::cos(kHomeLat*kRad);

struct Truth {
    double elapsed{}, latitude{},longitude{}, altitude{};
    double vn{},ve{},vd{},roll{},pitch{},yaw{};
    double wx{},wy{},wz{};
};
bool parse(const std::string& line,Truth& t) {
    std::array<double,13> v{};
    std::istringstream ss(line);
    std::string field;
    for(double& number : v) {
        if(!std::getline(ss,field,','))return false;
        try {
            std::size_t idx=0;
            number=std::stod(field,&idx);
            if(idx!=field.size() || !std::isfinite(number))return false;
        } catch(const std::exception&) {return false;}
    }
    if(std::getline(ss,field,','))return false;
    t={v[0],v[1],v[2],v[3],v[4],v[5],v[6],
       v[7],v[8],v[9],v[10],v[11],v[12]};
    return true;
}
}

int main(int argc,char** argv) {
    // NEVER attach this fixture to actual radar/INS: it reads SITL physics truth.
    if(argc!=2 || std::string(argv[1])!="--sitl-dynamic-fixture") {
        std::cerr<<"Refusing to run without --sitl-dynamic-fixture\n";
        return 2;
    }
    boat::sitl::TcpMavlinkLink link;
    if(!link.connectLocal(5762) || !link.awaitArduPilotHeartbeat(10000)) {
        std::cerr<<"Cannot verify Rover SITL on TCP 5762: "<<link.error()<<"\n";
        return 3;
    }
    std::cerr<<"SIM_TRUTH_FIXTURE_CONNECTED target_sysid="<<unsigned(link.systemId())<<"\n";
    std::cerr.flush();
    std::size_t sent=0;
    double previous_t=-1.0;
    double previous_n=0,previous_e=0;
    double max_jump=0;
    const auto last_data=std::chrono::steady_clock::now();
    (void)last_data;
    std::cout<<std::fixed<<std::setprecision(4);
    std::cout<<"time_s,physics_truth_n,physics_truth_e,physics_truth_vn,physics_truth_ve,"
                "outgoing_n,outgoing_e,synthetic_error_m,odometry_sent\n";
    while(true) {
        pollfd fd{0,POLLIN,0};
        const int ready=::poll(&fd,1,3000);
        if(ready<=0) {
            std::cerr<<"FAIL: physics truth source timeout\n";
            return 4;
        }
        std::string line;
        if(!std::getline(std::cin,line))break;
        if(line.empty())continue;
        Truth t{};
        if(!parse(line,t)) {
            std::cerr<<"FAIL: invalid SIM_STATE input (13 finite scalars expected)\n";
            return 5;
        }
        const double north=(t.latitude-kHomeLat)*kLatScale;
        const double east=(t.longitude-kHomeLon)*kLonScale;
        if(!std::isfinite(north) || !std::isfinite(east) ||
           std::abs(north)>5000 || std::abs(east)>5000 ||
           t.elapsed<=previous_t || t.elapsed>600 ||
           (previous_t>=0 && t.elapsed-previous_t>1.0) ||
           std::hypot(t.vn,t.ve)>10.0) {
            std::cerr<<"FAIL: invalid/expired/non-monotonic physics truth\n";
            return 6;
        }
        if(previous_t>=0) {
            const double displacement=std::hypot(north-previous_n,east-previous_e);
            max_jump=std::max(max_jump,displacement);
            if(displacement>std::max(1.2,12.0*(t.elapsed-previous_t))) {
                std::cerr<<"FAIL: discontinuous physics truth\n";
                return 7;
            }
        }
        boat::ValidatedOdometry6D sample{};
        sample.time_s=t.elapsed;
        // Deterministic bounded surrogate 'radar/map' error, fed from physics truth.
        // It is not an independently implemented shore radar measurement.
        const double error_n=0.12*std::sin(0.47*t.elapsed);
        const double error_e=0.12*std::cos(0.37*t.elapsed);
        sample.north_m=north+error_n;
        sample.east_m=east+error_e;
        sample.down_m=kHomeAltM-t.altitude;
        sample.north_mps=t.vn;sample.east_mps=t.ve;sample.down_mps=t.vd;
        sample.roll_rad=t.roll;sample.pitch_rad=t.pitch;sample.yaw_rad=t.yaw;
        sample.roll_rate_radps=t.wx;sample.pitch_rate_radps=t.wy;sample.yaw_rate_radps=t.wz;
        sample.position_sigma_m=0.6;
        sample.angular_sigma_rad=0.05;
        sample.quality=75;
        sample.origin_is_fixed_and_aligned=true;
        sample.vertical_state_valid=true;
        sample.full_attitude_valid=true;
        sample.independent_sensor_corrections_valid=true; // SIM-only synthetic reference
        if(!link.transmitOdometry(sample)) {
            std::cerr<<"FAIL: C++ ODOMETRY transmission: "<<link.error()<<"\n";
            return 8;
        }
        ++sent;
        std::cout<<t.elapsed<<','<<north<<','<<east<<','<<t.vn<<','<<t.ve<<','
                 <<sample.north_m<<','<<sample.east_m<<','
                 <<std::hypot(error_n,error_e)<<",1\n";
        std::cout.flush();
        previous_t=t.elapsed;
        previous_n=north;previous_e=east;
    }
    std::cerr<<"SITL_DYNAMIC_FIXTURE packets="<<sent<<" maximum_displacement_between_samples_m="
             <<max_jump<<" source=SIM_STATE not_GNSS=true not_real_INS=true\n";
    return sent>=10 ? 0:9;
}
