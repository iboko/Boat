#include "boat/sitl_link.hpp"

#ifdef BOAT_WITH_MAVLINK
#include <common/mavlink.h>
#endif

#include <algorithm>
#include <array>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstring>
#include <limits>
#include <string>

#if defined(__unix__) || defined(__APPLE__)
#include <arpa/inet.h>
#include <fcntl.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>
#endif

namespace boat::sitl {
namespace {
bool finite(double x) { return std::isfinite(x); }
double maximumAttitudeVariance(const ins::State& state) {
    return std::max({state.covariance[6*15+6],state.covariance[7*15+7],
                     state.covariance[8*15+8]});
}
}

TcpMavlinkLink::~TcpMavlinkLink() { close(); }
void TcpMavlinkLink::close() {
#if defined(__unix__) || defined(__APPLE__)
    if(socket_>=0) ::close(socket_);
#endif
    socket_=-1;
    autopilot_system_id_=0;
}

bool TcpMavlinkLink::connectLocal(std::uint16_t port, int timeout_ms) {
    close();
    error_.clear();
#if defined(__unix__) || defined(__APPLE__)
    if(port==0 || timeout_ms<=0 || timeout_ms>30000) {
        error_="invalid loopback connection settings"; return false;
    }
    const int fd=::socket(AF_INET,SOCK_STREAM,0);
    if(fd<0) { error_="cannot open TCP socket"; return false; }
    const int flags=::fcntl(fd,F_GETFL,0);
    if(flags<0 || ::fcntl(fd,F_SETFL,flags|O_NONBLOCK)<0) {
        error_="cannot configure TCP socket"; ::close(fd); return false;
    }
    sockaddr_in address{};
    address.sin_family=AF_INET;
    address.sin_port=htons(port);
    address.sin_addr.s_addr=htonl(INADDR_LOOPBACK);
    int ret=::connect(fd,reinterpret_cast<sockaddr*>(&address),sizeof(address));
    if(ret<0 && errno!=EINPROGRESS) {
        error_="loopback TCP connection refused"; ::close(fd); return false;
    }
    if(ret<0) {
        pollfd pfd{fd,POLLOUT,0};
        if(::poll(&pfd,1,timeout_ms)<=0 || !(pfd.revents&POLLOUT)) {
            error_="loopback TCP connection timed out"; ::close(fd); return false;
        }
        int status=0; socklen_t length=sizeof(status);
        if(::getsockopt(fd,SOL_SOCKET,SO_ERROR,&status,&length)<0 || status!=0) {
            error_="loopback TCP connection failed"; ::close(fd); return false;
        }
    }
    socket_=fd;
    return true;
#else
    (void)port; (void)timeout_ms;
    error_="SITL TCP link currently supports POSIX hosts only"; return false;
#endif
}

bool TcpMavlinkLink::awaitArduPilotHeartbeat(int timeout_ms) {
#if defined(BOAT_WITH_MAVLINK) && (defined(__unix__) || defined(__APPLE__))
    if(socket_<0 || timeout_ms<=0) { error_="no active SITL connection"; return false; }
    const auto end=std::chrono::steady_clock::now()+std::chrono::milliseconds(timeout_ms);
    mavlink_status_t status{};
    mavlink_message_t msg{};
    while(std::chrono::steady_clock::now()<end) {
        const auto remain=std::chrono::duration_cast<std::chrono::milliseconds>(
            end-std::chrono::steady_clock::now()).count();
        if(remain<=0) break;
        pollfd fd{socket_,POLLIN,0};
        const int r=::poll(&fd,1,static_cast<int>(std::min<long long>(remain,500)));
        if(r<0 && errno==EINTR)continue;
        if(r<0 || (fd.revents&(POLLHUP|POLLERR|POLLNVAL))) {
            error_="SITL TCP link disconnected"; close(); return false;
        }
        if(r==0)continue;
        std::array<std::uint8_t,4096> buffer{};
        const ssize_t n=::recv(socket_,buffer.data(),buffer.size(),0);
        if(n==0) { error_="SITL closed connection"; close(); return false; }
        if(n<0 && (errno==EAGAIN || errno==EINTR)) continue;
        if(n<0) { error_="SITL TCP read failed"; close(); return false; }
        for(ssize_t i=0;i<n;++i) {
            if(mavlink_frame_char_buffer(&msg,&status,buffer[static_cast<std::size_t>(i)],&msg,&status)
               != MAVLINK_FRAMING_OK) continue;
            if(msg.msgid!=MAVLINK_MSG_ID_HEARTBEAT || msg.sysid==0)continue;
            mavlink_heartbeat_t hb{};
            mavlink_msg_heartbeat_decode(&msg,&hb);
            if(hb.autopilot==MAV_AUTOPILOT_ARDUPILOTMEGA) {
                autopilot_system_id_=msg.sysid;
                return true;
            }
        }
    }
    error_="no verified ArduPilot heartbeat before deadline";
    return false;
#else
    (void)timeout_ms;
    error_="MAVLink/POSIX support is required";
    return false;
#endif
}

bool TcpMavlinkLink::transmitBytes(const std::uint8_t* p,std::size_t n) {
#if defined(__unix__) || defined(__APPLE__)
    if(socket_<0) return false;
    std::size_t sent=0;
    while(sent<n) {
        pollfd fd{socket_,POLLOUT,0};
        const int r=::poll(&fd,1,1000);
        if(r<=0 || (fd.revents&(POLLERR|POLLNVAL|POLLHUP))) {
            error_="TCP output timeout or disconnected"; close(); return false;
        }
        const ssize_t k=::send(socket_,p+sent,n-sent,MSG_NOSIGNAL);
        if(k<0 && (errno==EAGAIN || errno==EINTR))continue;
        if(k<=0) { error_="TCP output failed"; close(); return false; }
        sent+=static_cast<std::size_t>(k);
    }
    return true;
#else
    (void)p; (void)n;
    return false;
#endif
}

bool TcpMavlinkLink::transmitOdometry(const ValidatedOdometry6D& s) {
    if(!autopilotVerified()) { error_="refusing to transmit before autopilot heartbeat"; return false; }
    const auto bytes=ArduPilotOdometryEncoder::encode(s);
    if(bytes.empty()) { error_="ODOMETRY not eligible or MAVLink disabled"; return false; }
    return transmitBytes(bytes.data(),bytes.size());
}

bool makeOdometry(const ins::State& s,const ins::ImuSample& imu,double now_s,
                  const ExternalNavEvidence& proof,ValidatedOdometry6D& out) {
    out={};
    if(!proof.fixed_surveyed_origin || !proof.measured_vertical_position ||
       !proof.attitude_aligned || !s.origin_fixed_and_aligned || !imu.calibrated ||
       !finite(now_s) || !finite(imu.time_s) ||
       std::abs(s.time_s-imu.time_s)>0.025 ||
       !ins::assess(s,now_s,proof.gnss_denied).valid ||
       (proof.gnss_denied && !proof.shore_sensor_independent_of_gnss))return false;
    const auto q=ins::normalize(s.body_to_ned);
    const double sinp=2.0*(q.w*q.y-q.z*q.x);
    if(std::abs(sinp)>1.0+1e-6)return false;
    const double yaw=std::atan2(2.0*(q.w*q.z+q.x*q.y),1.0-2.0*(q.y*q.y+q.z*q.z));
    const double pitch=std::asin(std::clamp(sinp,-1.0,1.0));
    const double roll=std::atan2(2.0*(q.w*q.x+q.y*q.z),1.0-2.0*(q.x*q.x+q.y*q.y));
    const double sigma=ins::ErrorStateIns::horizontalSigmaM(s);
    const double angular_var=maximumAttitudeVariance(s);
    const double angular_sigma=std::sqrt(std::max(0.0,angular_var));
    if(!finite(sigma) || sigma>6.0 || !finite(angular_sigma) ||
       angular_sigma>0.25)return false;
    out.time_s=s.time_s;
    out.north_m=s.position_ned_m.x; out.east_m=s.position_ned_m.y; out.down_m=s.position_ned_m.z;
    out.north_mps=s.velocity_ned_mps.x; out.east_mps=s.velocity_ned_mps.y;
    out.down_mps=s.velocity_ned_mps.z;
    out.roll_rad=roll; out.pitch_rad=pitch; out.yaw_rad=yaw;
    const auto omega=imu.angular_rate_frd_radps-s.gyro_bias_frd_radps;
    out.roll_rate_radps=omega.x; out.pitch_rate_radps=omega.y; out.yaw_rate_radps=omega.z;
    out.position_sigma_m=std::max(0.01,sigma);
    out.angular_sigma_rad=std::max(0.001,angular_sigma);
    // Experimental quality scale, NOT a statistically calibrated integrity probability.
    out.quality=static_cast<std::uint8_t>(
        std::clamp(95-static_cast<int>(std::ceil(sigma*8.0)),1,95));
    out.origin_is_fixed_and_aligned=true;
    out.vertical_state_valid=true;
    out.full_attitude_valid=true;
    out.independent_sensor_corrections_valid=!proof.gnss_denied ||
                                            proof.shore_sensor_independent_of_gnss;
    return ArduPilotOdometryEncoder::eligible(out);
}
} // namespace boat::sitl
