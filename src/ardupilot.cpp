#include "boat/ardupilot.hpp"

#include <cmath>

#ifdef BOAT_WITH_MAVLINK
#include <common/mavlink.h>
#endif

namespace boat {
namespace {
bool finite(double x) { return std::isfinite(x); }
}

bool ArduPilotOdometryEncoder::eligible(const ValidatedOdometry6D& s) {
    return s.origin_is_fixed_and_aligned && s.vertical_state_valid &&
        s.full_attitude_valid && s.independent_sensor_corrections_valid &&
        s.quality > 0 && s.quality <= 100 && finite(s.time_s) && s.time_s >= 0.0 &&
        finite(s.north_m) && finite(s.east_m) && finite(s.down_m) &&
        finite(s.roll_rad) && finite(s.pitch_rad) && finite(s.yaw_rad) &&
        finite(s.north_mps) && finite(s.east_mps) && finite(s.down_mps) &&
        finite(s.roll_rate_radps) && finite(s.pitch_rate_radps) && finite(s.yaw_rate_radps) &&
        finite(s.position_sigma_m) && s.position_sigma_m > 0.0 &&
        finite(s.angular_sigma_rad) && s.angular_sigma_rad > 0.0;
}

std::vector<std::uint8_t> ArduPilotOdometryEncoder::encode(
    const ValidatedOdometry6D& s, std::uint8_t system_id, std::uint8_t component_id) {
    if (!eligible(s)) return {};
#ifdef BOAT_WITH_MAVLINK
    mavlink_odometry_t odom{};
    odom.time_usec = static_cast<std::uint64_t>(s.time_s * 1e6);
    // ArduPilot supports BODY_FRD and LOCAL_FRD for external ODOMETRY.
    // LOCAL_FRD origin MUST have fixed N/E/D alignment on this project.
    odom.frame_id = MAV_FRAME_LOCAL_FRD;
    odom.child_frame_id = MAV_FRAME_LOCAL_FRD;
    odom.x = static_cast<float>(s.north_m);
    odom.y = static_cast<float>(s.east_m);
    odom.z = static_cast<float>(s.down_m);
    const double cy = std::cos(s.yaw_rad / 2.0), sy = std::sin(s.yaw_rad / 2.0);
    const double cp = std::cos(s.pitch_rad / 2.0), sp = std::sin(s.pitch_rad / 2.0);
    const double cr = std::cos(s.roll_rad / 2.0), sr = std::sin(s.roll_rad / 2.0);
    odom.q[0] = static_cast<float>(cr * cp * cy + sr * sp * sy);
    odom.q[1] = static_cast<float>(sr * cp * cy - cr * sp * sy);
    odom.q[2] = static_cast<float>(cr * sp * cy + sr * cp * sy);
    odom.q[3] = static_cast<float>(cr * cp * sy - sr * sp * cy);
    odom.vx = static_cast<float>(s.north_mps);
    odom.vy = static_cast<float>(s.east_mps);
    odom.vz = static_cast<float>(s.down_mps);
    odom.rollspeed = static_cast<float>(s.roll_rate_radps);
    odom.pitchspeed = static_cast<float>(s.pitch_rate_radps);
    odom.yawspeed = static_cast<float>(s.yaw_rate_radps);
    const float pvar = static_cast<float>(s.position_sigma_m * s.position_sigma_m);
    const float avar = static_cast<float>(s.angular_sigma_rad * s.angular_sigma_rad);
    odom.pose_covariance[0] = pvar;
    odom.pose_covariance[6] = pvar;
    odom.pose_covariance[11] = pvar;
    odom.pose_covariance[15] = avar;
    odom.pose_covariance[18] = avar;
    odom.pose_covariance[20] = avar;
    odom.reset_counter = s.reset_counter;
    odom.quality = static_cast<std::int8_t>(s.quality);
    mavlink_message_t msg{};
    mavlink_msg_odometry_encode(system_id, component_id, &msg, &odom);
    std::vector<std::uint8_t> buffer(MAVLINK_MAX_PACKET_LEN);
    const std::uint16_t n = mavlink_msg_to_send_buffer(buffer.data(), &msg);
    buffer.resize(n);
    return buffer;
#else
    (void)system_id;
    (void)component_id;
    return {};
#endif
}
} // namespace boat
