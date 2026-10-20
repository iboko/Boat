#include "boat/sensor_fixture.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace boat::fixture {
namespace {
constexpr double kGravity=9.80665;
bool valid(double x) { return std::isfinite(x); }
bool valid(ins::Vec3 v) { return valid(v.x)&&valid(v.y)&&valid(v.z); }
bool valid(ins::Quaternion q) {
    return valid(q.w)&&valid(q.x)&&valid(q.y)&&valid(q.z);
}
bool dropout(double t,double from,double until) {
    return t>=from && t<until;
}
ins::Vec3 periodicNoise(double t,double scale) {
    return {scale*std::sin(0.83*t+0.31),
            scale*std::cos(0.53*t+0.77),
            0.45*scale*std::sin(0.37*t+0.42)};
}
}

SyntheticSensors::SyntheticSensors(Scenario s):scenario_(s) {
    if(!(s.radar_period_s>0.0 && s.dvl_period_s>0.0 &&
         s.radar_dropout_from_s<=s.radar_dropout_until_s &&
         s.bottom_dropout_from_s<=s.bottom_dropout_until_s &&
         valid(s.accel_bias_mps2)&&valid(s.gyro_bias_radps))) {
        throw std::invalid_argument("invalid synthetic sensor scenario");
    }
}

SensorSet SyntheticSensors::observe(const PhysicsFrame& t) {
    SensorSet result{};
    if(!valid(t.time_s)||!valid(t.position_ned_m)||
       !valid(t.ground_velocity_ned_mps)||!valid(t.body_to_ned)||
       !valid(t.angular_rate_frd_radps))return result;
    double dt=0.0;
    if(initialized_) {
        dt=t.time_s-previous_.time_s;
        if(!(dt>0.0 && dt<=0.7))return result;
    }
    const ins::Vec3 accel_ned=dt>0.0 ?
        (t.ground_velocity_ned_mps-previous_.ground_velocity_ned_mps)/dt :
        ins::Vec3{};
    // Accelerometer measures specific force, NOT gravity-corrected acceleration.
    const auto specific=ins::rotateInverse(
        ins::normalize(t.body_to_ned),accel_ned-ins::Vec3{0,0,kGravity});
    const auto accel_noise=periodicNoise(t.time_s,0.018);
    const auto gyro_noise=periodicNoise(t.time_s,0.00025);
    result.imu={t.time_s,
        specific+ins::Vec3{scenario_.accel_bias_mps2,
                           -0.7*scenario_.accel_bias_mps2,0.006}+accel_noise,
        t.angular_rate_frd_radps+
           ins::Vec3{0,0,scenario_.gyro_bias_radps}+gyro_noise,
        true};
    if(t.time_s-last_radar_time_s>=scenario_.radar_period_s-0.00001) {
        last_radar_time_s=t.time_s;
        if(dropout(t.time_s,scenario_.radar_dropout_from_s,
                             scenario_.radar_dropout_until_s)) {
            ++missing_shore_count_;
        } else {
            // Simulated surveyed shoreline position solution. No real radar scans.
            result.shore_fix={t.time_s,
                t.position_ned_m+periodicNoise(t.time_s,0.22),
                0.65,0.85,ins::AbsoluteSource::ShoreRadar,true,true};
            result.shore_fix_available=true;
            ++shore_count_;
        }
    }
    if(t.time_s-last_dvl_time_s>=scenario_.dvl_period_s-0.00001) {
        last_dvl_time_s=t.time_s;
        if(dropout(t.time_s,scenario_.bottom_dropout_from_s,
                             scenario_.bottom_dropout_until_s)) {
            ++missing_dvl_count_;
        } else {
            result.bottom_track={t.time_s,
                t.ground_velocity_ned_mps+periodicNoise(t.time_s,0.055),
                0.15,0.18,true,true};
            result.bottom_track_available=true;
            ++dvl_count_;
        }
    }
    previous_=t;
    initialized_=true;
    result.valid=true;
    return result;
}

SensorFusion::SensorFusion(Scenario s):sensors_(s) {}

bool SensorFusion::step(const PhysicsFrame& truth) {
    latest_=sensors_.observe(truth);
    if(!latest_.valid) {++rejected_steps_;return false;}
    if(first_) {
        if(!latest_.shore_fix_available || !latest_.bottom_track_available) {
            ++rejected_steps_;
            return false;
        }
        // Simulated initial surveyed position + surveyed heading / compass.
        // Orientation at startup still comes from physics truth in v0.7.
        estimator_.initialize(truth.time_s,latest_.shore_fix.position_ned_m,
             latest_.bottom_track.velocity_ned_mps,
             ins::normalize(truth.body_to_ned),true);
        if(!estimator_.correctPosition(latest_.shore_fix)) {
            ++rejected_steps_;return false;
        }
        if(!estimator_.correctGroundVelocity(latest_.bottom_track)) {
            ++rejected_steps_;return false;
        }
        first_=false;
        return true;
    }
    const double dt=latest_.imu.time_s-estimator_.state().time_s;
    if(!(dt>0.0 && dt<=0.7)) {++rejected_steps_;return false;}
    // ESKF's IMU step bound is 50ms. Divide each 10Hz physics sample into
    // short constant-specific-force/gyro integration intervals.
    const int steps=std::max(1,static_cast<int>(std::ceil(dt/0.025)));
    const double start=estimator_.state().time_s;
    for(int k=1;k<=steps;++k) {
        ins::ImuSample sample=latest_.imu;
        sample.time_s=(k==steps)?latest_.imu.time_s :
                       start+dt*static_cast<double>(k)/steps;
        if(!estimator_.propagate(sample)) {++rejected_steps_;return false;}
        ++propagated_;
    }
    if(latest_.bottom_track_available)
        (void)estimator_.correctGroundVelocity(latest_.bottom_track);
    if(latest_.shore_fix_available)
        (void)estimator_.correctPosition(latest_.shore_fix);
    return true;
}

bool SensorFusion::integrity() const {
    return ins::assess(estimator_.state(),estimator_.state().time_s,true).valid;
}
} // namespace boat::fixture
