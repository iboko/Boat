#include "boat/ardupilot.hpp"
#include <common/mavlink.h>

#include <cmath>
#include <iostream>
#include <stdexcept>
#include <vector>

void check(bool ok, const char* text) {
    if(!ok) throw std::runtime_error(text);
}

int main() {
    try {
        boat::ValidatedOdometry6D s{};
        s.time_s=12.5;
        s.north_m=4.0; s.east_m=8.0; s.down_m=0.5;
        s.yaw_rad=1.5707963267948966;
        s.north_mps=0.0; s.east_mps=2.0; s.down_mps=0.0;
        s.yaw_rate_radps=0.2;
        s.position_sigma_m=0.4; s.angular_sigma_rad=0.03;
        s.quality=90; s.reset_counter=2;
        s.origin_is_fixed_and_aligned=true;
        s.vertical_state_valid=true;
        s.full_attitude_valid=true;
        s.independent_sensor_corrections_valid=true;
        check(boat::ArduPilotOdometryEncoder::eligible(s),"valid synthetic 6D packet fixture");
        std::vector<std::uint8_t> data=boat::ArduPilotOdometryEncoder::encode(s);
        check(!data.empty(),"expected serialized MAVLink bytes");
        mavlink_message_t message{};
        mavlink_status_t status{};
        bool received=false;
        for(auto byte:data) {
            if(mavlink_parse_char(MAVLINK_COMM_0,byte,&message,&status)) {
                received=true;
            }
        }
        check(received && message.msgid==MAVLINK_MSG_ID_ODOMETRY,"ODOMETRY MAVLink2 packet parse");
        mavlink_odometry_t decoded{};
        mavlink_msg_odometry_decode(&message,&decoded);
        check(decoded.frame_id==MAV_FRAME_LOCAL_FRD &&
              decoded.child_frame_id==MAV_FRAME_BODY_FRD,
              "consistent LOCAL_FRD pose and BODY_FRD twist");
        check(std::abs(decoded.x-4.0)<1e-6 &&
              std::abs(decoded.y-8.0)<1e-6,"position encode");
        check(std::abs(decoded.vx-2.0)<1e-5 &&
              std::abs(decoded.vy)<1e-5,"NED ground velocity to FRD body");
        check(std::abs(decoded.pose_covariance[0]-0.16f)<1e-5,
              "position covariance must be variance");
        check(std::abs(decoded.pose_covariance[20]-0.0009f)<1e-6,
              "attitude covariance must be variance");
        check(std::isnan(decoded.velocity_covariance[0]),"unknown velocity covariance");
        check(decoded.quality==90 && decoded.reset_counter==2,"quality and reset");
        check(decoded.time_usec==12500000,"monotonic timestamp");
        s.full_attitude_valid=false;
        check(boat::ArduPilotOdometryEncoder::encode(s).empty(),
              "incomplete 3D attitude must not be transmitted");
        std::cout<<"PASS: MAVLink 2 ODOMETRY decode, NED/FRD twist and validity gates\n";
        return 0;
    } catch(const std::exception& e) {
        std::cerr<<"FAIL: "<<e.what()<<"\n";
        return 1;
    }
}
