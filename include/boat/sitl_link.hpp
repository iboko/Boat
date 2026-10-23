#pragma once

#include "boat/ardupilot.hpp"
#include "boat/ins.hpp"

#include <cstdint>
#include <string>

namespace boat::sitl {

// Loopback-only development link. This transport has no arming or control commands.
// ArduPilot SITL normally exposes an additional MAVLink TCP port at 5762 (SERIAL1).
class TcpMavlinkLink {
public:
    TcpMavlinkLink() = default;
    ~TcpMavlinkLink();
    TcpMavlinkLink(const TcpMavlinkLink&) = delete;
    TcpMavlinkLink& operator=(const TcpMavlinkLink&) = delete;
    bool connectLocal(std::uint16_t port, int timeout_ms = 3000);
    bool awaitArduPilotHeartbeat(int timeout_ms = 5000);
    bool transmitOdometry(const ValidatedOdometry6D& sample);
    bool connected() const { return socket_ >= 0; }
    bool autopilotVerified() const { return autopilot_system_id_ > 0; }
    std::uint8_t systemId() const { return autopilot_system_id_; }
    void close();
    const std::string& error() const { return error_; }

private:
    int socket_{-1};
    std::uint8_t autopilot_system_id_{0};
    std::string error_{};
    bool transmitBytes(const std::uint8_t* data, std::size_t size);
};

// Converts the experimental ESKF to a packet candidate only when explicitly
// verified geodetic datum, vertical observability, attitude alignment and
// current navigation integrity are known. This is not a sensor fusion driver.
struct ExternalNavEvidence {
    bool measured_vertical_position{false};
    bool attitude_aligned{false};
    bool fixed_surveyed_origin{false};
    bool shore_sensor_independent_of_gnss{false};
    bool gnss_denied{true};
    // Only for onboard IMU + bottom-track DVL + compass, after an accepted
    // real/simulated GNSS fix. This NEVER bypasses the time/covariance gate.
    bool bounded_onboard_dead_reckoning{false};
    double max_dead_reckoning_age_s{12.0};
};
bool makeOdometry(const ins::State& state, const ins::ImuSample& latest_imu,
                  double now_s, const ExternalNavEvidence& evidence,
                  ValidatedOdometry6D& output);
} // namespace boat::sitl
