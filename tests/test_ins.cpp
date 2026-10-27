#include "boat/ins.hpp"
#include "boat/ardupilot.hpp"

#include <array>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>

namespace {
void require(bool ok, const std::string& what) {
    if (!ok) throw std::runtime_error(what);
}
void near(double a, double b, double tolerance, const std::string& what) {
    require(std::abs(a-b)<=tolerance, what);
}
using boat::ins::Vec3;
void near3(Vec3 a, Vec3 b, double tolerance, const std::string& what) {
    near(boat::ins::norm(a-b),0,tolerance,what);
}
}

int main() {
    using namespace boat::ins;
    try {
        // Quaternion convention: body forward is North at identity and East at yaw +90.
        const auto eastFacing=expRotation({0,0,boat::kPi/2.0});
        near3(rotate(eastFacing,{1,0,0}),{0,1,0},1e-12,"NED frame rotation");
        near3(rotateInverse(eastFacing,{0,1,0}),{1,0,0},1e-12,"inverse body rotation");
        near(norm({3,4,0}),5,1e-12,"norm");
        const auto composed=multiply(expRotation({0,0,0.2}),expRotation({0,0,0.3}));
        near3(rotate(composed,{1,0,0}),{std::cos(0.5),std::sin(0.5),0},1e-10,
              "quaternion composition");

        ErrorStateIns ins;
        require(!ins.propagate({0.01,{0,0,-9.80665},{0,0,0},true}),
                "reject propagation before initialization");
        ins.initialize(0.0,{0,0,0},{0,0,0},{1,0,0,0},true);
        require(!assess(ins.state(),0.0,false).valid,"initialization is not absolute fix");
        require(!ins.propagate({0.01,{0,0,-9.80665},{0,0,0},false}),
                "require calibrated IMU");
        require(!ins.propagate({0.1,{0,0,-9.80665},{0,0,0},true}),
                "reject missing IMU epochs");
        require(!ins.propagate({0.01,{151,0,0},{0,0,0},true}),
                "reject saturated accelerometer measurement");
        require(!ins.propagate({0.01,{0,0,-9.80665},{13,0,0},true}),
                "reject excessive angular rate");
        near(ins.state().time_s,0.0,0.0,"rejected IMU must not advance time");

        for(int i=1;i<=1000;++i) {
            const double t=i*0.01;
            require(ins.propagate({t,{0,0,-9.80665},{0,0,0},true}),"stationary propagation");
        }
        near3(ins.state().position_ned_m,{0,0,0},1e-7,"stationary INS position");
        near3(ins.state().velocity_ned_mps,{0,0,0},1e-7,"gravity cancellation");
        require(ins.state().covariance[0]>0 &&
                ins.state().covariance[16]>0, "covariance propagation");
        require(ErrorStateIns::maximumCovarianceAsymmetry(ins.state()) < 1e-10,
                "symmetric covariance propagation");
        require(!ins.correctGroundVelocity({10,{0,0,0},0.1,0.1,false,true}),
                "water-relative speed cannot correct ground velocity");
        require(ins.correctGroundVelocity({10,{0,0,0},0.1,0.1,true,true}),
                "bottom-track Doppler correction");
        require(!ins.correctPosition({10,{0.5,0,0},1.0,2.0,
                                      AbsoluteSource::ShoreRadar,true,false}),
                "reject shore pose secretly derived from GNSS");
        require(!ins.correctPosition({10,{1000,0,0},0.1,0.1,
                                      AbsoluteSource::Gnss,true,false,
                                      {10,3,1.1,true,false}}),
                "reject position innovation outlier");
        require(ins.correctPosition({10,{0.2,-0.2,0},1.0,2.0,
                                     AbsoluteSource::ShoreRadar,true,true}),
                "accept independent shore radar fix");
        require(ins.state().accepted_fixes==1 && ins.state().rejected_fixes>=2,
                "fix acceptance/rejection counters");
        require(assess(ins.state(),10,true).valid,"GNSS denied but independent fix available");
        require(!assess(ins.state(),45,true).valid,"deny stale INS or shore fix");
        require(!assess(ins.state(),10.5,true).valid,"deny stale state");
        require(ErrorStateIns::maximumCovarianceAsymmetry(ins.state())<1e-10,
                "symmetric Joseph covariance");
        require(ins.state().covariance[0]>0.0,"Joseph update retains variance");
        near(norm({ins.state().body_to_ned.x,ins.state().body_to_ned.y,
                   ins.state().body_to_ned.z}),
             0.0,0.2,"stationary attitude correction bounded");

        const auto snapshot=makePlanarSnapshot(ins.state());
        require(snapshot.initialized && std::isfinite(snapshot.pos_variance_m2),
                "Planar snapshot for route planning");
        require(!snapshot.last_fix_was_gnss,"source attribution");
        // A horizontal right-body specific acceleration from an East-oriented vessel
        // must produce ground acceleration towards East.
        ErrorStateIns accelerating;
        accelerating.initialize(0,{0,0,0},{0,0,0},eastFacing,true);
        for(int i=1;i<=100;++i) {
            require(accelerating.propagate({i*0.01,{1,0,-9.80665},{0,0,0},true}),
                    "accelerating INS");
        }
        near(accelerating.state().velocity_ned_mps.y,1.0,1e-8,
             "East-facing acceleration velocity");
        near(accelerating.state().position_ned_m.y,0.5,1e-8,
             "East-facing acceleration displacement");
        near(accelerating.state().position_ned_m.z,0,1e-8,
             "leveled height remains constant");

        // Genuine GNSS works, but absence of non-GNSS georeferencing
        // blocks navigation in GNSS-denied policy.
        ErrorStateIns gnssOnly;
        gnssOnly.initialize(0,{0,0,0},{0,0,0},{1,0,0,0},true);
        require(!gnssOnly.correctPosition({0,{0.1,0.1,0},0.8,1.5,
                                           AbsoluteSource::Gnss,true,false}),
                "missing GNSS receiver quality accepted");
        require(!gnssOnly.correctPosition({0,{0.1,0.1,0},0.8,1.5,
                                           AbsoluteSource::Gnss,true,false,
                                           {4,2,6.0,false,true}}),
                "invalid or interfered GNSS quality accepted");
        require(!std::isfinite(gnssOnly.state().last_gnss_fix_s),
                "rejected GNSS quality refreshed fix timestamp");
        require(gnssOnly.correctPosition({0,{0.1,0.1,0},0.8,1.5,
                                          AbsoluteSource::Gnss,true,false,
                                          {10,3,1.1,true,false}}),
                "valid GNSS correction");
        require(assess(gnssOnly.state(),0,false).valid,"GNSS allowed mode");
        require(!assess(gnssOnly.state(),0,true).valid,
                "GNSS alone cannot satisfy no-GNSS requirement");

        Configuration invalid_cfg{};
        invalid_cfg.max_attitude_nis=0.0;
        bool invalid_config_rejected=false;
        try { (void)ErrorStateIns{invalid_cfg}; }
        catch(const std::invalid_argument&) {invalid_config_rejected=true;}
        require(invalid_config_rejected,"invalid attitude gate accepted");

        ErrorStateIns notSurveyed;
        notSurveyed.initialize(0,{0,0,0},{0,0,0},{1,0,0,0},false);
        require(!notSurveyed.propagate({0.01,{0,0,-9.80665},{0,0,0},true}),
                "unaligned origin blocks propagation");
        require(!notSurveyed.correctPosition({0,{0,0,0},1.0,1.0,
                                               AbsoluteSource::ShoreRadar,true,true}),
                "unaligned origin blocks correction");

        // A 2D demo must never be sent as fabricated 6-DoF MAVLink ODOMETRY.
        require(!boat::ArduPilotOdometryEncoder::eligible({}),
                "no invented 6D external-navigation solution");
        std::cout << "PASS: 3D INS gravity, attitude, covariance, velocity, "
                     "innovation gates and GNSS-denied integrity\n";
    } catch(const std::exception& error) {
        std::cerr << "FAIL: " << error.what() << '\n';
        return 1;
    }
}
