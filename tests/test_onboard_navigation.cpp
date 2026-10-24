#include "boat/sensor_fixture.hpp"
#include "boat/sitl_link.hpp"

#include <cmath>
#include <iostream>
#include <stdexcept>

namespace {
void require(bool b, const char* msg) {
    if(!b) throw std::runtime_error(msg);
}
boat::fixture::PhysicsFrame frame(double t) {
    // Ground truth is used ONLY for generating the three virtual sensor
    // channels and test error measurement; never passed as a position fix.
    return {t,{t,0,0},{1,0,0},{1,0,0,0},{0,0,0}};
}
}

int main() {
    using namespace boat::fixture;
    try {
        Scenario scenario{};
        scenario.reference_mode=ReferenceMode::OnboardDeadReckoning;
        SensorFusion short_outage(scenario);
        for(int i=0;i<=300;++i) {
            const double t=i*0.1;
            require(short_outage.step(frame(t)),"onboard GNSS/INS/DVL progression failed");
            if(t>10.5 && t<13.0)
                require(short_outage.integrity(),"short GNSS outage must be coastable");
        }
        require(short_outage.generator().gnssCount()>15,"GNSS data not generated");
        require(short_outage.generator().missedGnssCount()>0,"short GNSS outage missing");
        require(short_outage.generator().shoreCount()==0,"onboard mode used shore radar");
        require(short_outage.state().last_non_gnss_fix_s<0.0,
                "bottom DVL or compass masqueraded as an absolute fix");
        require(short_outage.state().last_gnss_fix_s>25.0,
                "GNSS recovery not accepted");
        require(short_outage.integrity(),"GNSS recovery did not restore integrity");
        boat::sitl::ExternalNavEvidence onboard_proof{};
        onboard_proof.fixed_surveyed_origin=true;
        onboard_proof.measured_vertical_position=true;
        onboard_proof.attitude_aligned=true;
        onboard_proof.gnss_denied=true;
        onboard_proof.bounded_onboard_dead_reckoning=true;
        onboard_proof.max_dead_reckoning_age_s=12.0;
        boat::ValidatedOdometry6D onboard_odom{};
        require(boat::sitl::makeOdometry(short_outage.state(),
                    short_outage.sensors().imu,30.0,onboard_proof,onboard_odom),
                "valid onboard fusion must produce external odometry");
        onboard_proof.max_dead_reckoning_age_s=0.0;
        require(!boat::sitl::makeOdometry(short_outage.state(),
                    short_outage.sensors().imu,30.0,onboard_proof,onboard_odom),
                "unbounded/zero-age odometry policy must be rejected");
        const auto& p=short_outage.state().position_ned_m;
        require(std::hypot(p.x-30.0,p.y)<2.0,
                "onboard estimated position diverged in simple fixture");

        // The integration runner schedules this outage only after Rover
        // has confirmed ARMED + AUTO. Coasting must remain bounded and
        // the renewed GNSS corrections must subsequently be accepted.
        Scenario post_arm=scenario;
        post_arm.gnss_dropout_from_s=600.0;
        post_arm.gnss_dropout_until_s=600.0;
        SensorFusion active_route(post_arm);
        unsigned missed_before=0;
        for(int i=0;i<=350;++i) {
            const double t=i*0.1;
            require(active_route.step(frame(t)),
                    "post-ARM short-outage C++ estimator rejected sample");
            if(i==200) {
                missed_before=active_route.generator().missedGnssCount();
                require(active_route.beginGnssBriefOutageAfterArm(),
                        "post-ARM brief GNSS dropout not accepted");
            }
            if(i>=200 && i<=260)
                require(active_route.integrity(),
                        "brief GNSS loss must preserve bounded route integrity");
        }
        require(active_route.generator().missedGnssCount()>missed_before,
                "post-ARM brief GNSS loss did not suppress any GNSS fixes");
        require(active_route.briefGnssOutageCompleted(),
                "post-ARM GNSS loss was not followed by an accepted GNSS fix");
        require(!active_route.beginGnssBriefOutageAfterArm(),
                "a second post-ARM brief outage directive was accepted");
        require(active_route.state().last_gnss_fix_s>30.0,
                "GNSS did not reacquire after post-ARM brief loss");
        require(active_route.generator().shoreCount()==0,
                "post-ARM route unexpectedly required shoreline reflectors");

        Scenario lost=scenario;
        lost.gnss_dropout_from_s=10.0;
        lost.gnss_dropout_until_s=200.0;
        lost.max_dead_reckoning_s=4.0;
        SensorFusion long_outage(lost);
        bool became_invalid=false;
        for(int i=0;i<=210;++i) {
            const double t=i*0.1;
            require(long_outage.step(frame(t)),"long-outage ESKF step failed");
            if(t>=15.0 && !long_outage.integrity())became_invalid=true;
        }
        require(became_invalid && !long_outage.integrity(),
                "stale GNSS must stop external navigation even with DVL");
        onboard_proof.max_dead_reckoning_age_s=4.0;
        require(!boat::sitl::makeOdometry(long_outage.state(),
                    long_outage.sensors().imu,21.0,onboard_proof,onboard_odom),
                "stale GNSS must block actual MAVLink odometry output");
        require(long_outage.generator().shoreCount()==0,
                "long outage depended on surveyed reflectors");

        Scenario no_initial_fix=scenario;
        no_initial_fix.gnss_dropout_from_s=0.0;
        no_initial_fix.gnss_dropout_until_s=200.0;
        SensorFusion unaligned(no_initial_fix);
        require(!unaligned.step(frame(0.0)) && !unaligned.integrity(),
                "uninitialized no-shore INS must fail closed without GNSS origin");
        require(!unaligned.beginGnssBriefOutageAfterArm(),
                "brief outage was accepted before GNSS origin initialization");

        Scenario water=lost;
        water.bottom_dropout_from_s=9.0;
        water.bottom_dropout_until_s=200.0;
        SensorFusion no_bottom(water);
        for(int i=0;i<=200;++i)
            require(no_bottom.step(frame(i*0.1)),
                    "water-track-only fixture must still propagate inertial state");
        require(no_bottom.generator().waterModeCount()>0,
                "water-track outage mode not simulated");
        require(!no_bottom.integrity(),
                "water velocity cannot reset the dead-reckoning time budget");

        const auto& s=short_outage.state();
        require(!boat::ins::assessDeadReckoning(s,s.time_s,
                    {6.0,0.0,0.2}).valid,
                "invalid zero coasting policy was accepted");
        std::cout<<"PASS: onboard GNSS origin -> IMU/compass/bottom DVL ESKF; "
                   "short GNSS coast, reacquisition and long-outage stop; "
                   "no shoreline, no water-track masquerade\n";
        return 0;
    } catch(const std::exception& e) {
        std::cerr<<"FAIL: "<<e.what()<<'\n';
        return 1;
    }
}
