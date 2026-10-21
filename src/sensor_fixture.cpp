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
         valid(s.accel_bias_mps2)&&valid(s.gyro_bias_radps)&&
         valid(s.modeled_current_n_mps)&&valid(s.modeled_current_e_mps)&&
         valid(s.max_non_gnss_fix_age_s)&&s.max_non_gnss_fix_age_s>=1.0&&
         s.max_non_gnss_fix_age_s<=30.0)) {
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
    // SIM_STATE gyro channels can be defined in a different basis from its
    // Euler angles in the Gazebo JSON/FDM bridge. For the SITL fixture derive
    // BODY angular velocity from successive surveyed truth quaternions, then
    // add independent synthetic gyro bias/noise. This is still a virtual IMU.
    ins::Vec3 gyro_body{};
    if(dt>0.0) {
        const auto oldq=ins::normalize(previous_.body_to_ned);
        const auto newq=ins::normalize(t.body_to_ned);
        const ins::Quaternion inverse{oldq.w,-oldq.x,-oldq.y,-oldq.z};
        auto dq=ins::normalize(ins::multiply(inverse,newq));
        if(dq.w<0.0)
            dq={-dq.w,-dq.x,-dq.y,-dq.z};
        const ins::Vec3 axis{dq.x,dq.y,dq.z};
        const double magnitude=ins::norm(axis);
        gyro_body=magnitude>1.e-12 ?
            axis*(2.0*std::atan2(magnitude,dq.w)/(magnitude*dt)) :
            axis*(2.0/dt);
    }
    const auto accel_noise=periodicNoise(t.time_s,0.018);
    const auto gyro_noise=periodicNoise(t.time_s,0.00025);
    result.imu={t.time_s,
        specific+ins::Vec3{scenario_.accel_bias_mps2,
                           -0.7*scenario_.accel_bias_mps2,0.006}+accel_noise,
        gyro_body+ins::Vec3{0,0,scenario_.gyro_bias_radps}+gyro_noise,
        true};
    // Separate virtual compass/inclinometer observation, with a small
    // known angle noise, rather than direct attitude assignment in ODOMETRY.
    // It is synthesized from Gazebo truth; no real sensor is claimed.
    const auto tilt_noise=periodicNoise(t.time_s,0.004);
    result.attitude={t.time_s,
        ins::normalize(ins::multiply(t.body_to_ned,
                                     ins::expRotation(tilt_noise))),
        0.025,true,true};
    result.attitude_fix_available=true;
    if(t.time_s-last_radar_time_s>=scenario_.radar_period_s-0.00001) {
        last_radar_time_s=t.time_s;
        if(dropout(t.time_s,scenario_.radar_dropout_from_s,
                             scenario_.radar_dropout_until_s)) {
            ++missing_shore_count_;
        } else {
            // The generator may only output UNLABELED radar ranges.
            // Landmark association is done against the estimated ESKF pose,
            // not against physics truth.
            result.unlabelled_echoes=ShoreMapAssociator::simulateEchoes(
                t.position_ned_m,t.time_s);
            if(result.unlabelled_echoes.valid) {
                ++shore_count_;
                if(t.time_s>=3.0 && t.time_s<5.0)
                    ++range_outlier_count_;
            } else ++missing_shore_count_;
        }
    }
    if(t.time_s-last_dvl_time_s>=scenario_.dvl_period_s-0.00001) {
        last_dvl_time_s=t.time_s;
        if(dropout(t.time_s,scenario_.bottom_dropout_from_s,
                             scenario_.bottom_dropout_until_s)) {
            // Water-track DVL is still available, but it measures vessel
            // velocity RELATIVE TO WATER. Unknown current forbids treating
            // these readings as Earth/ground velocity corrections.
            result.water_track={t.time_s,
                t.ground_velocity_ned_mps-
                  ins::Vec3{scenario_.modeled_current_n_mps,
                            scenario_.modeled_current_e_mps,0.0}+
                  periodicNoise(t.time_s,0.055),
                0.17,0.20,false,true};
            result.water_track_available=true;
            ++water_mode_count_;
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

SensorFusion::SensorFusion(Scenario s):sensors_(s),
    max_fix_age_s_(s.max_non_gnss_fix_age_s) {}

bool SensorFusion::step(const PhysicsFrame& truth) {
    latest_=sensors_.observe(truth);
    if(!latest_.valid) {++rejected_steps_;return false;}
    if(first_) {
        if(!latest_.unlabelled_echoes.valid || !latest_.bottom_track_available) {
            ++rejected_steps_;
            return false;
        }
        // Only survey datum/vertical position and initial heading come
        // from SITL test initialization. Horizontal position is computed
        // solely by solving raw shoreline reflector ranges.
        const auto associated=ShoreMapAssociator::associate(
            latest_.unlabelled_echoes,{0,0,truth.position_ned_m.z});
        if(!associated.valid){++rejected_steps_;return false;}
        latest_.shore_ranges=associated.ranges;
        latest_.unassociated_echoes=associated.unmatched;
        const auto survey=ShoreRangeModel::solve(
            latest_.shore_ranges,{0,0,truth.position_ned_m.z});
        if(!survey.valid) {++rejected_steps_;return false;}
        latest_.range_inliers=survey.inliers;
        latest_.shore_fix={truth.time_s,survey.position_ned_m,
            0.65,0.85,ins::AbsoluteSource::ShoreRadar,true,true};
        latest_.shore_fix_available=true;
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
    if(latest_.attitude_fix_available) {
        if(estimator_.correctAttitude(latest_.attitude))
            ++accepted_attitudes_;
        else
            ++rejected_attitudes_;
    }
    if(latest_.bottom_track_available)
        (void)estimator_.correctGroundVelocity(latest_.bottom_track);
    // Water-track velocity is explicitly NOT passed to correctGroundVelocity.
    // No current estimator exists in this version.
    if(latest_.unlabelled_echoes.valid) {
        const auto associated=ShoreMapAssociator::associate(
            latest_.unlabelled_echoes,estimator_.state().position_ned_m);
        if(associated.valid) {
            latest_.shore_ranges=associated.ranges;
            latest_.unassociated_echoes=associated.unmatched;
            const auto solution=ShoreRangeModel::solve(
                associated.ranges,estimator_.state().position_ned_m);
            if(solution.valid) {
                latest_.range_inliers=solution.inliers;
                latest_.shore_fix={truth.time_s,solution.position_ned_m,
                    0.65,0.85,ins::AbsoluteSource::ShoreRadar,true,true};
                latest_.shore_fix_available=
                    estimator_.correctPosition(latest_.shore_fix);
            }
        }
    }
    return true;
}

bool SensorFusion::integrity() const {
    ins::IntegrityPolicy policy{};
    policy.max_independent_fix_age_during_gnss_denial_s=max_fix_age_s_;
    return ins::assess(estimator_.state(),estimator_.state().time_s,
                       true,policy).valid;
}
} // namespace boat::fixture
