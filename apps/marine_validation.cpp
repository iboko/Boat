#include "boat/marine_dynamics.hpp"
#include "boat/sensor_fixture.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {
double bound(double v,double min,double max){return std::clamp(v,min,max);}
double planar(boat::ins::Vec3 a,boat::ins::Vec3 b){
    return std::hypot(a.x-b.x,a.y-b.y);
}
double wrap(double a){
    return std::atan2(std::sin(a),std::cos(a));
}
void require(bool b,const char* msg){if(!b)throw std::runtime_error(msg);}
}

int main(int argc,char** argv){
    if(argc!=4 || std::string(argv[1])!="--sitl-only-marine-surrogate" ||
       (std::string(argv[2])!="route" && std::string(argv[2])!="blackout")) {
        std::cerr<<"Usage: --sitl-only-marine-surrogate route|blackout output.csv\n";
        return 2;
    }
    const bool blackout=std::string(argv[2])=="blackout";
    try {
        boat::marine::MarineParameters parameters{};
        boat::marine::MarineEnvironment environment{
            {0.35,-0.20,0},{2.0,3.0,0}};
        boat::marine::MarineThreeDof vessel(parameters,environment);
        boat::fixture::Scenario scenario{};
        if(blackout){
            scenario.radar_dropout_from_s=6.0;
            scenario.radar_dropout_until_s=500.0;
            scenario.max_non_gnss_fix_age_s=3.0;
        }
        boat::fixture::SensorFusion navigator(scenario);
        const std::array<boat::ins::Vec3,3> targets{{
            {12.0,0.0,0.0},{12.0,12.0,0.0},{0.0,12.0,0.0}}};
        std::array<double,3> nearest{1.e10,1.e10,1.e10};
        unsigned target=0;
        bool neutral=false;
        bool integrity_stop=false;
        double neutral_start=0.0;
        boat::ins::Vec3 neutral_at{};
        double max_estimation_error=0.0;
        std::vector<double> errors;
        std::ofstream csv(argv[3]);
        require(bool(csv),"CSV output unavailable");
        csv<<std::fixed<<std::setprecision(6);
        csv<<"time_s,physics_n,physics_e,eskf_n,eskf_e,"
              "ground_vn,ground_ve,water_body_forward,water_body_starboard,"
              "heading_rad,thrust_n,yaw_moment_nm,target_index,"
              "nav_integrity,neutral_propulsion,estimate_error_m\n";
        constexpr double dt=0.02;
        boat::marine::MarineControl command{};
        for(int k=0;k<15000;++k) {
            const auto& state=vessel.state();
            // The 3DOF physics truth is ONLY consumed by the measurement
            // generator and an evaluation logger. Guidance uses ESKF state.
            if(k%5==0){
                boat::fixture::PhysicsFrame truth{
                    state.time_s,state.position_ned_m,
                    state.velocity_ground_ned_mps,
                    boat::ins::expRotation({0,0,state.heading_rad}),
                    {0,0,state.yaw_rate_radps}};
                require(navigator.step(truth),"C++ ESKF propagation failed");
                const auto& nav=navigator.state();
                const double error=planar(nav.position_ned_m,state.position_ned_m);
                max_estimation_error=std::max(max_estimation_error,error);
                errors.push_back(error);
                for(unsigned j=0;j<3;++j)
                    nearest[j]=std::min(nearest[j],
                                 planar(targets[j],state.position_ned_m));
                if(!neutral && !navigator.integrity()){
                    integrity_stop=true;
                    neutral=true;
                    neutral_start=state.time_s;
                    neutral_at=state.position_ned_m;
                    std::cerr<<"MARINE_NAVIGATION_INVALID time_s="<<neutral_start
                             <<" mode=NEUTRAL_THRUST not_station_keeping\n";
                }
                if(!neutral && !blackout && target<targets.size() &&
                   planar(targets[target],nav.position_ned_m)<1.65) {
                    ++target;
                    if(target==targets.size()){
                        neutral=true;
                        neutral_start=state.time_s;
                        neutral_at=state.position_ned_m;
                        std::cerr<<"MARINE_ROUTE_COMPLETE time_s="<<neutral_start<<"\n";
                    }
                }
                if(!neutral){
                    const auto difference=targets[target]-nav.position_ned_m;
                    const double heading_desired=std::atan2(difference.y,difference.x);
                    const auto axis=boat::ins::rotate(nav.body_to_ned,{1,0,0});
                    const double heading_est=std::atan2(axis.y,axis.x);
                    const double yaw_err=wrap(heading_desired-heading_est);
                    const double yawrate=navigator.sensors().imu.angular_rate_frd_radps.z;
                    const double distance=std::hypot(difference.x,difference.y);
                    const double desired_ground_speed=bound(0.45*distance,0.25,1.5);
                    const double nav_speed=std::hypot(nav.velocity_ned_mps.x,
                                                     nav.velocity_ned_mps.y);
                    command.forward_thrust_n=bound(
                        32.0+50.0*(desired_ground_speed-nav_speed),0,115);
                    command.yaw_moment_nm=bound(
                        45.0*yaw_err-24.0*yawrate,-48,48);
                }else {
                    // Loss of navigation does not imply fixed geographic position.
                    // This deliberately models a neutral-thrust response.
                    command={0,0};
                }
                csv<<state.time_s<<','<<state.position_ned_m.x<<','
                   <<state.position_ned_m.y<<','
                   <<nav.position_ned_m.x<<','<<nav.position_ned_m.y<<','
                   <<state.velocity_ground_ned_mps.x<<','
                   <<state.velocity_ground_ned_mps.y<<','
                   <<state.velocity_through_water_body_mps.x<<','
                   <<state.velocity_through_water_body_mps.y<<','
                   <<state.heading_rad<<','<<command.forward_thrust_n<<','
                   <<command.yaw_moment_nm<<','<<target<<','
                   <<int(navigator.integrity())<<','<<int(neutral)<<','
                   <<error<<"\n";
            }
            require(vessel.step(dt,command),"marine hydrodynamic step failed");
            if(neutral && vessel.state().time_s-neutral_start>=20.0)break;
        }
        require(neutral,"neutral phase never reached");
        const double drift=planar(vessel.state().position_ned_m,neutral_at);
        const unsigned reached=target;
        std::sort(errors.begin(),errors.end());
        const double p95=errors[static_cast<std::size_t>((errors.size()-1)*0.95)];
        require(p95<3.0,"synthetic marine ESKF error exceeded 3m P95");
        require(drift>2.0,"neutral thrust falsely treated as station keeping");
        if(blackout){
            require(integrity_stop,"shore blackout did not trigger integrity policy");
            require(reached<3,"blackout test completed route before loss");
        } else {
            require(!integrity_stop,"normal route lost navigation integrity");
            require(reached==3,"three marine surrogate waypoints were not reached");
            for(double n:nearest)require(n<2.5,"true hull missed a waypoint");
        }
        std::cout<<std::setprecision(3)<<std::fixed
                 <<"PASS_MARINE_"<<(blackout?"BLACKOUT":"ROUTE")
                 <<" waypoints="<<reached
                 <<" drift_after_neutral_20s_m="<<drift
                 <<" eskf_p95_m="<<p95
                 <<" max_eskf_error_m="<<max_estimation_error
                 <<" stopped_for_integrity="<<int(integrity_stop)
                 <<" current_n=0.35 current_e=-0.20 wind_n=2 wind_e=3"
                 <<" model=unvalidated_3dof_surrogate NOT_GAZEBO=true\n";
        return 0;
    }catch(const std::exception& e){
        std::cerr<<"FAIL_MARINE_TRIAL: "<<e.what()<<"\n";
        return 1;
    }
}
