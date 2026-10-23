#include "boat/sitl_link.hpp"
#include "boat/sensor_fixture.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <fstream>
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
    const bool shore=(argc>=2 &&
        std::string(argv[1])=="--sitl-sensor-fusion-fixture");
    const bool onboard=(argc>=2 &&
        std::string(argv[1])=="--sitl-onboard-navigation-fixture");
    const bool baseline=argc==2 && (shore || onboard);
    const bool blackout=(argc==5 && (shore || onboard) &&
        std::string(argv[2])==(onboard?"--gnss-blackout":"--radar-blackout") &&
        std::string(argv[3])=="--hold-marker");
    if(!baseline && !blackout) {
        std::cerr<<"Refusing non-SITL call: "
                    "--sitl-sensor-fusion-fixture [--radar-blackout --hold-marker FILE] "
                    "OR --sitl-onboard-navigation-fixture "
                    "[--gnss-blackout --hold-marker FILE]\n";
        return 2;
    }
    boat::sitl::TcpMavlinkLink link;
    if(!link.connectLocal(5762) || !link.awaitArduPilotHeartbeat(10000)) {
        std::cerr<<"Cannot verify Rover SITL on TCP 5762: "<<link.error()<<"\n";
        return 3;
    }
    std::cerr<<"SYNTHETIC_SENSORS_ESKF_CONNECTED target_sysid="<<unsigned(link.systemId())<<"\n";
    std::cerr.flush();
    boat::fixture::Scenario scenario{};
    if(onboard) {
        scenario.reference_mode=boat::fixture::ReferenceMode::OnboardDeadReckoning;
    }
    if(blackout) {
        // No test blackout before independently confirmed ARM+AUTO.
        if(onboard) {
            scenario.gnss_dropout_from_s=600.0;
            scenario.gnss_dropout_until_s=600.0;
            scenario.max_dead_reckoning_s=4.0;
        } else {
            scenario.radar_dropout_from_s=600.0;
            scenario.radar_dropout_until_s=600.0;
            scenario.max_non_gnss_fix_age_s=3.0;
        }
    }
    boat::fixture::SensorFusion fusion{scenario};
    bool hold_marker_written=false;
    bool blackout_activated=false;
    std::size_t sent=0;
    std::size_t rejected_odometry=0;
    double max_estimation_error=0.0;
    double previous_t=-1.0;
    double previous_n=0,previous_e=0;
    double max_jump=0;
    const auto last_data=std::chrono::steady_clock::now();
    (void)last_data;
    std::cout<<std::fixed<<std::setprecision(4);
    std::cout<<"time_s,physics_truth_n,physics_truth_e,physics_truth_vn,physics_truth_ve,"
                "eskf_n,eskf_e,estimate_error_m,odometry_sent,radar_fix,dvl_fix,"
                "eskf_position_sigma_m,water_track,shore_ranges,shore_inliers,"
                "raw_unlabeled_echoes,unmatched_echoes,nav_integrity,gnss_fix\n";
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
        if(line=="RADAR_BLACKOUT" || line=="GNSS_BLACKOUT") {
            const bool right_command=onboard ? line=="GNSS_BLACKOUT" :
                                               line=="RADAR_BLACKOUT";
            if(!blackout || blackout_activated || !right_command ||
               !(onboard ? fusion.beginGnssBlackoutAfterArm() :
                           fusion.beginRadarBlackoutAfterArm())) {
                std::cerr<<"FAIL: invalid post-ARM sensor blackout directive\n";
                return 12;
            }
            blackout_activated=true;
            std::cerr<<(onboard?"GNSS_BLACKOUT":"RADAR_BLACKOUT")
                     <<"_AFTER_ARM_CONFIRMED now_s="
                     <<fusion.state().time_s<<"\n";
            continue;
        }
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
        // SIM_STATE exists ONLY inside the synthetic sensor generator.
        // The actual ODOMETRY is computed from the 15-state C++ ESKF,
        // never assigned from ground-truth latitude/longitude or velocity.
        const boat::ins::Quaternion attitude=boat::ins::normalize(
            boat::ins::multiply(
                boat::ins::multiply(
                    boat::ins::expRotation({0,0,t.yaw}),
                    boat::ins::expRotation({0,t.pitch,0})),
                boat::ins::expRotation({t.roll,0,0})));
        const boat::fixture::PhysicsFrame truth{
            t.elapsed,{north,east,kHomeAltM-t.altitude},
            {t.vn,t.ve,t.vd},attitude,{t.wx,t.wy,t.wz}};
        if(!fusion.step(truth)) {
            std::cerr<<"FAIL: C++ INS synthetic sensor integration rejected sample\n";
            return 10;
        }
        const auto& estimate=fusion.state();
        boat::sitl::ExternalNavEvidence evidence{};
        evidence.measured_vertical_position=true;
        evidence.attitude_aligned=true; // initial attitude from SIM fixture
        evidence.fixed_surveyed_origin=true; // simulated fixed NED origin
        evidence.shore_sensor_independent_of_gnss=!onboard;
        evidence.gnss_denied=true;
        evidence.bounded_onboard_dead_reckoning=onboard;
        evidence.max_dead_reckoning_age_s=fusion.maxDeadReckoningAgeS();
        boat::ValidatedOdometry6D sample{};
        const bool safe=fusion.integrity();
        if(blackout && !safe && !hold_marker_written) {
            std::ofstream marker(argv[4],std::ios::trunc);
            if(!marker) {
                std::cerr<<"FAIL: cannot create required SITL HOLD marker\n";
                return 11;
            }
            marker<<"INVALID_NAVIGATION time_s="<<t.elapsed
                  <<(onboard?" reason=dead_reckoning_limit_exceeded\n":
                              " reason=shore_map_fix_stale\n");
            marker.close();
            std::cerr<<"NAVIGATION_INVALID_REQUIRES_HOLD time_s="
                     <<t.elapsed<<"\n";
            hold_marker_written=true;
        }
        const bool eligible=safe && boat::sitl::makeOdometry(
            estimate,fusion.sensors().imu,t.elapsed,evidence,sample);
        if(!eligible)++rejected_odometry;
        else {
            if(!link.transmitOdometry(sample)) {
                std::cerr<<"FAIL: ESKF ODOMETRY transmission: "<<link.error()<<"\n";
                return 8;
            }
            ++sent;
        }
        const double error=std::hypot(
            estimate.position_ned_m.x-north,
            estimate.position_ned_m.y-east);
        max_estimation_error=std::max(max_estimation_error,error);
        std::cout<<t.elapsed<<','<<north<<','<<east<<','<<t.vn<<','<<t.ve<<','
                 <<estimate.position_ned_m.x<<','<<estimate.position_ned_m.y<<','
                 <<error<<','<<int(eligible)<<','
                 <<int(fusion.sensors().shore_fix_available)<<','
                 <<int(fusion.sensors().bottom_track_available)<<','
                 <<boat::ins::ErrorStateIns::horizontalSigmaM(estimate)<<','
                 <<int(fusion.sensors().water_track_available)<<','
                 <<fusion.sensors().shore_ranges.count<<','
                 <<fusion.sensors().range_inliers<<','
                 <<fusion.sensors().unlabelled_echoes.count<<','
                 <<fusion.sensors().unassociated_echoes<<','
                 <<int(safe)<<','<<int(fusion.sensors().gnss_fix_available)<<"\n";
        std::cout.flush();
        previous_t=t.elapsed;
        previous_n=north;previous_e=east;
    }
    std::cerr<<"SITL_SENSOR_ESKF packets="<<sent
             <<" blocked="<<rejected_odometry
             <<" radar_fixes="<<fusion.generator().shoreCount()
             <<" dvl_fixes="<<fusion.generator().dvlCount()
             <<" radar_dropouts="<<fusion.generator().missedShoreCount()
             <<" gnss_fixes="<<fusion.generator().gnssCount()
             <<" gnss_dropouts="<<fusion.generator().missedGnssCount()
             <<" dvl_dropouts="<<fusion.generator().missedDvlCount()
             <<" water_mode="<<fusion.generator().waterModeCount()
             <<" radar_range_outliers="<<fusion.generator().radarOutlierCount()
             <<" max_eskf_truth_error_m="<<max_estimation_error
             <<" attitude_aided="<<fusion.acceptedAttitudes()
             <<" attitude_rejected="<<fusion.rejectedAttitudes()
             <<" safety_blackout_triggered="<<int(hold_marker_written)
             <<" source="<<(onboard?"GNSS_IMU_compass_bottom_DVL_ESKF":
                      "anonymous_radar_map_association_IMU_DVL_ESKF")
             <<" simulated_sensors=true\n";
    const bool missed_source=onboard ?
        fusion.generator().missedGnssCount()>0 :
        fusion.generator().missedShoreCount()>0;
    return sent>=10 && missed_source &&
        (blackout ? (hold_marker_written && blackout_activated) :
            fusion.generator().missedDvlCount()>0) ? 0:9;
}
