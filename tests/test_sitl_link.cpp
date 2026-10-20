#include "boat/sitl_link.hpp"
#include <common/mavlink.h>

#include <arpa/inet.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>

#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <thread>

namespace {
void check(bool value,const char* what) {
    if(!value)throw std::runtime_error(what);
}
}

int main() {
    using namespace boat;
    using namespace boat::sitl;
    using namespace boat::ins;
    try {
        ErrorStateIns estimator;
        estimator.initialize(0,{0,0,0},{0,0,0},{1,0,0,0},true);
        check(estimator.correctPosition({0,{0,0,0},0.8,1.0,
                  AbsoluteSource::ShoreRadar,true,true}),"synthetic shore fix");
        const ImuSample imu{0.01,{0,0,-9.80665},{0,0,0},true};
        check(estimator.propagate(imu),"propagate INS");
        ExternalNavEvidence proof{};
        proof.measured_vertical_position=true;
        proof.attitude_aligned=true;
        proof.fixed_surveyed_origin=true;
        proof.shore_sensor_independent_of_gnss=true;
        proof.gnss_denied=true;
        ValidatedOdometry6D odom;
        check(makeOdometry(estimator.state(),imu,0.01,proof,odom),"valid fixture");
        check(!makeOdometry(estimator.state(),imu,1.1,proof,odom),"stale state rejected");
        proof.shore_sensor_independent_of_gnss=false;
        check(!makeOdometry(estimator.state(),imu,0.01,proof,odom),
              "GNSS-denial requires independent fixes");
        proof.shore_sensor_independent_of_gnss=true;
        proof.measured_vertical_position=false;
        check(!makeOdometry(estimator.state(),imu,0.01,proof,odom),
              "vertical state not invented");
        proof.measured_vertical_position=true;
        check(makeOdometry(estimator.state(),imu,0.01,proof,odom),"restore valid sample");

        int listener=::socket(AF_INET,SOCK_STREAM,0);
        check(listener>=0,"create fixture socket");
        sockaddr_in address{};
        address.sin_family=AF_INET;
        address.sin_addr.s_addr=htonl(INADDR_LOOPBACK);
        address.sin_port=0;
        check(::bind(listener,reinterpret_cast<sockaddr*>(&address),sizeof(address))==0,
              "bind fixture socket");
        check(::listen(listener,1)==0,"listen fixture");
        socklen_t size=sizeof(address);
        check(::getsockname(listener,reinterpret_cast<sockaddr*>(&address),&size)==0,
              "read port");
        const auto port=ntohs(address.sin_port);

        std::atomic<bool> saw_odometry{false};
        std::thread server([&] {
            int client=::accept(listener,nullptr,nullptr);
            if(client<0)return;
            timeval timeout{5,0};
            ::setsockopt(client,SOL_SOCKET,SO_RCVTIMEO,&timeout,sizeof(timeout));
            mavlink_message_t heartbeat{};
            mavlink_msg_heartbeat_pack(1,1,&heartbeat,MAV_TYPE_SURFACE_BOAT,
                 MAV_AUTOPILOT_ARDUPILOTMEGA,0,0,MAV_STATE_STANDBY);
            std::array<std::uint8_t,MAVLINK_MAX_PACKET_LEN> buffer{};
            const auto count=mavlink_msg_to_send_buffer(buffer.data(),&heartbeat);
            if(::send(client,buffer.data(),count,0)!=count) {
                ::close(client); return;
            }
            mavlink_message_t msg{};
            mavlink_status_t status{};
            for(int attempt=0;attempt<8 && !saw_odometry.load();++attempt) {
                const auto got=::recv(client,buffer.data(),buffer.size(),0);
                if(got<=0)break;
                for(ssize_t i=0;i<got;++i) {
                    if(mavlink_parse_char(MAVLINK_COMM_3,buffer[static_cast<std::size_t>(i)],
                                        &msg,&status) &&
                       msg.msgid==MAVLINK_MSG_ID_ODOMETRY) {
                        mavlink_odometry_t decoded{};
                        mavlink_msg_odometry_decode(&msg,&decoded);
                        if(decoded.quality>0 && decoded.frame_id==MAV_FRAME_LOCAL_FRD &&
                           decoded.child_frame_id==MAV_FRAME_BODY_FRD &&
                           std::abs(decoded.x)<0.1) saw_odometry=true;
                    }
                }
            }
            ::close(client);
        });
        {
            TcpMavlinkLink link;
            check(link.connectLocal(port),"TCP client loopback connection");
            check(!link.transmitOdometry(odom),"require verified remote autopilot");
            check(link.awaitArduPilotHeartbeat(3000),"MAVLink heartbeat handshake");
            check(link.systemId()==1,"autopilot system identity");
            check(link.transmitOdometry(odom),"send valid ODOMETRY");
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        server.join();
        ::close(listener);
        check(saw_odometry.load(),"fixture received and decoded MAVLink2 ODOMETRY");
        std::cout<<"PASS: local TCP companion handshake, GNSS-denied integrity gate "
                    "and ODOMETRY packet reception\n";
        return 0;
    } catch(const std::exception& error) {
        std::cerr<<"FAIL: "<<error.what()<<"\n";
        return 1;
    }
}
