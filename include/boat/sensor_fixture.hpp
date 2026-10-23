#pragma once
#include "boat/ins.hpp"
#include "boat/shore_ranges.hpp"
#include "boat/shore_association.hpp"

namespace boat::fixture {

enum class ReferenceMode { ShoreAided, OnboardDeadReckoning };

// Strictly SITL: physics truth produces three separately-noised sensor channels.
// Do not use this as a physical sensor driver or as evidence of independence.
struct PhysicsFrame {
    double time_s{};
    ins::Vec3 position_ned_m{};
    ins::Vec3 ground_velocity_ned_mps{};
    ins::Quaternion body_to_ned{};
    ins::Vec3 angular_rate_frd_radps{};
};

struct SensorSet {
    ins::ImuSample imu{};
    ins::AbsoluteAttitude attitude{};
    bool attitude_fix_available{false};
    ins::AbsoluteFix shore_fix{};
    ins::AbsoluteFix gnss_fix{};
    ShoreRanges shore_ranges{};
    AnonymousRanges unlabelled_echoes{};
    unsigned unassociated_echoes{};
    unsigned range_inliers{};
    ins::GroundVelocity bottom_track{};
    ins::GroundVelocity water_track{};
    bool water_track_available{false};
    bool shore_fix_available{false};
    bool gnss_fix_available{false};
    bool bottom_track_available{false};
    bool valid{false};
};

struct Scenario {
    ReferenceMode reference_mode{ReferenceMode::ShoreAided};
    double gnss_period_s{1.0};
    double gnss_dropout_from_s{10.0};
    double gnss_dropout_until_s{13.0};
    double max_dead_reckoning_s{12.0};
    double max_dead_reckoning_horizontal_sigma_m{6.0};
    double radar_period_s{0.7};
    double dvl_period_s{0.3};
    double radar_dropout_from_s{9.0};
    double radar_dropout_until_s{13.0};
    double bottom_dropout_from_s{11.0};
    double bottom_dropout_until_s{13.0};
    double accel_bias_mps2{0.012};
    double gyro_bias_radps{0.0007};
    double modeled_current_n_mps{0.35};
    double modeled_current_e_mps{-0.20};
    double max_non_gnss_fix_age_s{6.0};
};

class SyntheticSensors {
public:
    explicit SyntheticSensors(Scenario scenario = {});
    SensorSet observe(const PhysicsFrame& truth);
    void beginRadarBlackoutAfter(double now_s, double delay_s=6.0);
    void beginGnssBlackoutAfter(double now_s, double delay_s=6.0);
    bool initialized() const { return initialized_; }
    unsigned shoreCount() const { return shore_count_; }
    unsigned gnssCount() const { return gnss_count_; }
    unsigned missedGnssCount() const { return missing_gnss_count_; }
    unsigned dvlCount() const { return dvl_count_; }
    unsigned missedShoreCount() const { return missing_shore_count_; }
    unsigned missedDvlCount() const { return missing_dvl_count_; }
    unsigned waterModeCount() const { return water_mode_count_; }
    unsigned radarOutlierCount() const { return range_outlier_count_; }
    unsigned clutterDiscardCount() const { return clutter_discard_count_; }

private:
    Scenario scenario_{};
    PhysicsFrame previous_{};
    double last_radar_time_s{-1.0e10};
    double last_gnss_time_s{-1.0e10};
    double last_dvl_time_s{-1.0e10};
    bool initialized_{false};
    unsigned shore_count_{0}, dvl_count_{0};
    unsigned gnss_count_{0}, missing_gnss_count_{0};
    unsigned missing_shore_count_{0}, missing_dvl_count_{0};
    unsigned water_mode_count_{0}, range_outlier_count_{0};
    unsigned clutter_discard_count_{0};
};

// Interface accepts no GNSS input. First fix/attitude are synthetic
// survey/compass initializations; subsequent navigation uses the ESKF.
class SensorFusion {
public:
    explicit SensorFusion(Scenario scenario = {});
    bool step(const PhysicsFrame& truth);
    const ins::State& state() const { return estimator_.state(); }
    const SensorSet& sensors() const { return latest_; }
    bool integrity() const;
    bool beginRadarBlackoutAfterArm();
    bool beginGnssBlackoutAfterArm();
    ReferenceMode referenceMode() const { return mode_; }
    double maxDeadReckoningAgeS() const { return dead_reckoning_limit_s_; }
    double lastShoreFixTimeS() const { return estimator_.state().last_non_gnss_fix_s; }
    unsigned propagated() const { return propagated_; }
    unsigned acceptedAttitudes() const { return accepted_attitudes_; }
    unsigned rejectedAttitudes() const { return rejected_attitudes_; }
    unsigned rejectedSteps() const { return rejected_steps_; }
    const SyntheticSensors& generator() const { return sensors_; }

private:
    SyntheticSensors sensors_;
    ins::ErrorStateIns estimator_;
    ReferenceMode mode_{ReferenceMode::ShoreAided};
    double max_fix_age_s_{6.0};
    double dead_reckoning_limit_s_{12.0};
    double dead_reckoning_sigma_limit_m_{6.0};
    SensorSet latest_{};
    bool first_{true};
    unsigned propagated_{0}, rejected_steps_{0};
    unsigned accepted_attitudes_{0}, rejected_attitudes_{0};
};

} // namespace boat::fixture
