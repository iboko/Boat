#pragma once
#include "boat/ins.hpp"

namespace boat::fixture {

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
    ins::AbsoluteFix shore_fix{};
    ins::GroundVelocity bottom_track{};
    bool shore_fix_available{false};
    bool bottom_track_available{false};
    bool valid{false};
};

struct Scenario {
    double radar_period_s{0.7};
    double dvl_period_s{0.3};
    double radar_dropout_from_s{9.0};
    double radar_dropout_until_s{13.0};
    double bottom_dropout_from_s{11.0};
    double bottom_dropout_until_s{13.0};
    double accel_bias_mps2{0.012};
    double gyro_bias_radps{0.0007};
};

class SyntheticSensors {
public:
    explicit SyntheticSensors(Scenario scenario = {});
    SensorSet observe(const PhysicsFrame& truth);
    bool initialized() const { return initialized_; }
    unsigned shoreCount() const { return shore_count_; }
    unsigned dvlCount() const { return dvl_count_; }
    unsigned missedShoreCount() const { return missing_shore_count_; }
    unsigned missedDvlCount() const { return missing_dvl_count_; }

private:
    Scenario scenario_{};
    PhysicsFrame previous_{};
    double last_radar_time_s{-1.0e10};
    double last_dvl_time_s{-1.0e10};
    bool initialized_{false};
    unsigned shore_count_{0}, dvl_count_{0};
    unsigned missing_shore_count_{0}, missing_dvl_count_{0};
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
    unsigned propagated() const { return propagated_; }
    unsigned rejectedSteps() const { return rejected_steps_; }
    const SyntheticSensors& generator() const { return sensors_; }

private:
    SyntheticSensors sensors_;
    ins::ErrorStateIns estimator_;
    SensorSet latest_{};
    bool first_{true};
    unsigned propagated_{0}, rejected_steps_{0};
};

} // namespace boat::fixture
