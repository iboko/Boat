#include "boat/ins.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace boat::ins {
namespace {
constexpr int N = 15;
constexpr double kTiny = 1e-14;
double& cell(Covariance15& m, int r, int c) { return m[static_cast<std::size_t>(r * N + c)]; }
double cell(const Covariance15& m, int r, int c) { return m[static_cast<std::size_t>(r * N + c)]; }
bool finite(double x) { return std::isfinite(x); }
bool finite(Vec3 v) { return finite(v.x) && finite(v.y) && finite(v.z); }
bool finite(Quaternion q) { return finite(q.w) && finite(q.x) && finite(q.y) && finite(q.z); }
double squared(double x) { return x * x; }
void diagonal(Covariance15& m, int offset, double sigma) {
    for (int i = 0; i < 3; ++i) cell(m, offset + i, offset + i) = squared(sigma);
}
void rotationMatrix(Quaternion q, double r[3][3]) {
    for (int j = 0; j < 3; ++j) {
        const auto rotated = rotate(q, {double(j == 0), double(j == 1), double(j == 2)});
        r[0][j] = rotated.x; r[1][j] = rotated.y; r[2][j] = rotated.z;
    }
}
void skew(Vec3 v, double a[3][3]) {
    a[0][0] = 0.0; a[0][1] = -v.z; a[0][2] = v.y;
    a[1][0] = v.z; a[1][1] = 0.0; a[1][2] = -v.x;
    a[2][0] = -v.y; a[2][1] = v.x; a[2][2] = 0.0;
}
bool inverse3(const double a[3][3], double inv[3][3]) {
    const double c00 = a[1][1]*a[2][2] - a[1][2]*a[2][1];
    const double c01 = a[1][2]*a[2][0] - a[1][0]*a[2][2];
    const double c02 = a[1][0]*a[2][1] - a[1][1]*a[2][0];
    const double det = a[0][0]*c00 + a[0][1]*c01 + a[0][2]*c02;
    if (!finite(det) || det <= kTiny) return false;
    inv[0][0]=c00/det; inv[0][1]=(a[0][2]*a[2][1]-a[0][1]*a[2][2])/det;
    inv[0][2]=(a[0][1]*a[1][2]-a[0][2]*a[1][1])/det;
    inv[1][0]=c01/det; inv[1][1]=(a[0][0]*a[2][2]-a[0][2]*a[2][0])/det;
    inv[1][2]=(a[0][2]*a[1][0]-a[0][0]*a[1][2])/det;
    inv[2][0]=c02/det; inv[2][1]=(a[0][1]*a[2][0]-a[0][0]*a[2][1])/det;
    inv[2][2]=(a[0][0]*a[1][1]-a[0][1]*a[1][0])/det;
    return true;
}
} // namespace

Vec3 operator+(Vec3 a, Vec3 b) { return {a.x+b.x, a.y+b.y, a.z+b.z}; }
Vec3 operator-(Vec3 a, Vec3 b) { return {a.x-b.x, a.y-b.y, a.z-b.z}; }
Vec3 operator*(Vec3 a, double s) { return {a.x*s, a.y*s, a.z*s}; }
Vec3 operator/(Vec3 a, double s) { return {a.x/s, a.y/s, a.z/s}; }
double dot(Vec3 a, Vec3 b) { return a.x*b.x + a.y*b.y + a.z*b.z; }
double norm(Vec3 a) { return std::sqrt(dot(a, a)); }
Vec3 cross(Vec3 a, Vec3 b) {
    return {a.y*b.z-a.z*b.y, a.z*b.x-a.x*b.z, a.x*b.y-a.y*b.x};
}
Quaternion multiply(Quaternion a, Quaternion b) {
    return {a.w*b.w-a.x*b.x-a.y*b.y-a.z*b.z,
            a.w*b.x+a.x*b.w+a.y*b.z-a.z*b.y,
            a.w*b.y-a.x*b.z+a.y*b.w+a.z*b.x,
            a.w*b.z+a.x*b.y-a.y*b.x+a.z*b.w};
}
Quaternion normalize(Quaternion q) {
    const double d = std::sqrt(q.w*q.w+q.x*q.x+q.y*q.y+q.z*q.z);
    if (!finite(d) || d < kTiny) throw std::invalid_argument("invalid attitude quaternion");
    return {q.w/d,q.x/d,q.y/d,q.z/d};
}
Quaternion expRotation(Vec3 radians) {
    const double theta = norm(radians);
    const double scale = theta < 1e-8 ? 0.5 - theta*theta/48.0 : std::sin(theta/2.0)/theta;
    return {std::cos(theta/2.0), radians.x*scale, radians.y*scale, radians.z*scale};
}
Vec3 rotate(Quaternion q, Vec3 v) {
    const Vec3 u{q.x,q.y,q.z};
    const Vec3 t = cross(u,v)*2.0;
    return v + t*q.w + cross(u,t);
}
Vec3 rotateInverse(Quaternion q, Vec3 v) { return rotate({q.w,-q.x,-q.y,-q.z},v); }

ErrorStateIns::ErrorStateIns(Configuration c) : cfg_(c) {
    if (!finite(c.max_imu_dt_s) || c.max_imu_dt_s <= 0.0 ||
        !finite(c.max_measurement_age_s) || c.max_measurement_age_s < 0.0 ||
        !finite(c.gravity_ned_mps2) || c.gravity_ned_mps2 <= 0.0 ||
        !finite(c.accel_white_noise_mps2_per_sqrt_hz) || c.accel_white_noise_mps2_per_sqrt_hz <= 0.0 ||
        !finite(c.gyro_white_noise_radps_per_sqrt_hz) || c.gyro_white_noise_radps_per_sqrt_hz <= 0.0 ||
        !finite(c.accel_bias_walk_mps3_per_sqrt_hz) || c.accel_bias_walk_mps3_per_sqrt_hz <= 0.0 ||
        !finite(c.gyro_bias_walk_radps2_per_sqrt_hz) || c.gyro_bias_walk_radps2_per_sqrt_hz <= 0.0 ||
        !finite(c.max_position_nis) || c.max_position_nis <= 0.0 ||
        !finite(c.max_velocity_nis) || c.max_velocity_nis <= 0.0 ||
        !finite(c.initial_position_sigma_m) || c.initial_position_sigma_m <= 0.0 ||
        !finite(c.initial_velocity_sigma_mps) || c.initial_velocity_sigma_mps <= 0.0 ||
        !finite(c.initial_attitude_sigma_rad) || c.initial_attitude_sigma_rad <= 0.0 ||
        !finite(c.initial_accel_bias_sigma_mps2) || c.initial_accel_bias_sigma_mps2 <= 0.0 ||
        !finite(c.initial_gyro_bias_sigma_radps) || c.initial_gyro_bias_sigma_radps <= 0.0)
        throw std::invalid_argument("invalid INS configuration");
}

void ErrorStateIns::initialize(double t, Vec3 p, Vec3 v, Quaternion q, bool aligned) {
    if (!finite(t) || !finite(p) || !finite(v) || !finite(q))
        throw std::invalid_argument("invalid INS initialization");
    state_ = {};
    state_.initialized = true;
    state_.origin_fixed_and_aligned = aligned;
    state_.time_s = t; state_.position_ned_m = p;
    state_.velocity_ned_mps = v; state_.body_to_ned = normalize(q);
    diagonal(state_.covariance,0,cfg_.initial_position_sigma_m);
    diagonal(state_.covariance,3,cfg_.initial_velocity_sigma_mps);
    diagonal(state_.covariance,6,cfg_.initial_attitude_sigma_rad);
    diagonal(state_.covariance,9,cfg_.initial_accel_bias_sigma_mps2);
    diagonal(state_.covariance,12,cfg_.initial_gyro_bias_sigma_radps);
}

bool ErrorStateIns::propagate(const ImuSample& m) {
    if (!state_.initialized || !state_.origin_fixed_and_aligned ||
        !m.calibrated || !finite(m.time_s) ||
        !finite(m.specific_force_frd_mps2) || !finite(m.angular_rate_frd_radps)) return false;
    const double dt = m.time_s - state_.time_s;
    if (!(dt > 0.0 && dt <= cfg_.max_imu_dt_s)) return false;
    const Vec3 w = m.angular_rate_frd_radps - state_.gyro_bias_frd_radps;
    const Vec3 f = m.specific_force_frd_mps2 - state_.accel_bias_frd_mps2;
    const Quaternion qmid = normalize(multiply(state_.body_to_ned,expRotation(w*(dt/2.0))));
    const Vec3 a = rotate(qmid,f) + Vec3{0.0,0.0,cfg_.gravity_ned_mps2};
    state_.position_ned_m = state_.position_ned_m + state_.velocity_ned_mps*dt + a*(0.5*dt*dt);
    state_.velocity_ned_mps = state_.velocity_ned_mps + a*dt;
    state_.body_to_ned = normalize(multiply(state_.body_to_ned,expRotation(w*dt)));

    double rot[3][3], sx[3][3], sw[3][3];
    rotationMatrix(qmid,rot); skew(f,sx); skew(w,sw);
    Covariance15 phi{};
    for (int i=0; i<N; ++i) cell(phi,i,i)=1.0;
    for (int i=0; i<3; ++i) {
        cell(phi,i,3+i) = dt;
        cell(phi,6+i,12+i) = -dt;
        for (int j=0; j<3; ++j) {
            double sum=0;
            for (int k=0; k<3; ++k) sum+=rot[i][k]*sx[k][j];
            cell(phi,3+i,6+j)=-dt*sum;
            cell(phi,3+i,9+j)=-dt*rot[i][j];
            cell(phi,6+i,6+j)-=dt*sw[i][j];
        }
    }
    Covariance15 temp{}, updated{};
    for (int i=0; i<N; ++i)
        for (int k=0; k<N; ++k) {
            const double ph=cell(phi,i,k);
            if (ph==0.0) continue;
            for (int j=0; j<N; ++j) cell(temp,i,j)+=ph*cell(state_.covariance,k,j);
        }
    for (int i=0; i<N; ++i)
        for (int k=0; k<N; ++k) {
            const double a_ik=cell(temp,i,k);
            if (a_ik==0.0) continue;
            for (int j=0; j<N; ++j) cell(updated,i,j)+=a_ik*cell(phi,j,k);
        }
    const double qa=squared(cfg_.accel_white_noise_mps2_per_sqrt_hz);
    const double qg=squared(cfg_.gyro_white_noise_radps_per_sqrt_hz);
    for (int i=0; i<3; ++i) {
        cell(updated,i,i) += qa*dt*dt*dt/3.0;
        cell(updated,i,3+i) += qa*dt*dt/2.0;
        cell(updated,3+i,i) += qa*dt*dt/2.0;
        cell(updated,3+i,3+i) += qa*dt;
        cell(updated,6+i,6+i) += qg*dt;
        cell(updated,9+i,9+i) += squared(cfg_.accel_bias_walk_mps3_per_sqrt_hz)*dt;
        cell(updated,12+i,12+i) += squared(cfg_.gyro_bias_walk_radps2_per_sqrt_hz)*dt;
    }
    for(int i=0;i<N;++i)
        for(int j=0;j<i;++j) {
            const double vmean=(cell(updated,i,j)+cell(updated,j,i))/2.0;
            cell(updated,i,j)=vmean; cell(updated,j,i)=vmean;
        }
    state_.covariance=updated;
    state_.time_s=m.time_s;
    return true;
}

bool ErrorStateIns::measurementUpdate(Vec3 r, const std::array<double,3>& variances,
                                       int offset, double gate) {
    const double z[3]{r.x,r.y,r.z};
    double s[3][3]{}, inv[3][3]{};
    for (int i=0;i<3;++i)
        for(int j=0;j<3;++j)
            s[i][j]=cell(state_.covariance,offset+i,offset+j)+(i==j?variances[i]:0.0);
    if(!inverse3(s,inv))return false;
    double nis=0;
    for(int i=0;i<3;++i)
        for(int j=0;j<3;++j) nis+=z[i]*inv[i][j]*z[j];
    if(!finite(nis) || nis>gate)return false;
    double K[N][3]{};
    for(int i=0;i<N;++i)
        for(int a=0;a<3;++a)
            for(int b=0;b<3;++b)K[i][a]+=cell(state_.covariance,i,offset+b)*inv[b][a];
    double delta[N]{};
    for(int i=0;i<N;++i)
        for(int a=0;a<3;++a)delta[i]+=K[i][a]*z[a];

    // Joseph stabilized update: (I-KH)P(I-KH)^T+KRK^T.
    Covariance15 updated{};
    for(int i=0;i<N;++i)
        for(int j=0;j<N;++j) {
            double v=cell(state_.covariance,i,j);
            for(int a=0;a<3;++a) {
                v-=K[i][a]*cell(state_.covariance,offset+a,j);
                v-=cell(state_.covariance,i,offset+a)*K[j][a];
                for(int b=0;b<3;++b) v+=K[i][a]*s[a][b]*K[j][b];
            }
            cell(updated,i,j)=v;
        }
    state_.position_ned_m=state_.position_ned_m+Vec3{delta[0],delta[1],delta[2]};
    state_.velocity_ned_mps=state_.velocity_ned_mps+Vec3{delta[3],delta[4],delta[5]};
    const Vec3 da{delta[6],delta[7],delta[8]};
    state_.body_to_ned=normalize(multiply(state_.body_to_ned,expRotation(da)));
    state_.accel_bias_frd_mps2=state_.accel_bias_frd_mps2+Vec3{delta[9],delta[10],delta[11]};
    state_.gyro_bias_frd_radps=state_.gyro_bias_frd_radps+Vec3{delta[12],delta[13],delta[14]};
    // First-order reset Jacobian for right-invariant small-angle error.
    double ds[3][3]; skew(da,ds);
    Covariance15 reset{};
    for(int i=0;i<N;++i)cell(reset,i,i)=1.0;
    for(int i=0;i<3;++i)
        for(int j=0;j<3;++j) cell(reset,6+i,6+j)-=0.5*ds[i][j];
    Covariance15 temp{}, result{};
    for(int i=0;i<N;++i)
        for(int k=0;k<N;++k) {
            const double a=cell(reset,i,k);
            if(a==0.0)continue;
            for(int j=0;j<N;++j)cell(temp,i,j)+=a*cell(updated,k,j);
        }
    for(int i=0;i<N;++i)
        for(int k=0;k<N;++k) {
            const double a=cell(temp,i,k);
            if(a==0.0)continue;
            for(int j=0;j<N;++j)cell(result,i,j)+=a*cell(reset,j,k);
        }
    for(int i=0;i<N;++i)
        for(int j=0;j<i;++j) {
            const double mean=(cell(result,i,j)+cell(result,j,i))*0.5;
            cell(result,i,j)=mean; cell(result,j,i)=mean;
        }
    state_.covariance=result;
    return true;
}

bool ErrorStateIns::correctPosition(const AbsoluteFix& f) {
    if (!state_.initialized || !state_.origin_fixed_and_aligned || !f.valid ||
        !finite(f.time_s) || !finite(f.position_ned_m) ||
        !finite(f.horizontal_sigma_m) || !finite(f.vertical_sigma_m) ||
        f.horizontal_sigma_m<=0.0 || f.vertical_sigma_m<=0.0 ||
        std::abs(state_.time_s-f.time_s)>cfg_.max_measurement_age_s ||
        (f.source!=AbsoluteSource::Gnss && !f.independently_georeferenced)) {
        ++state_.rejected_fixes; return false;
    }
    const std::array<double,3> variance{squared(f.horizontal_sigma_m),
                                        squared(f.horizontal_sigma_m),
                                        squared(f.vertical_sigma_m)};
    if (!measurementUpdate(f.position_ned_m-state_.position_ned_m,
                           variance,0,cfg_.max_position_nis)) {
        ++state_.rejected_fixes; return false;
    }
    state_.last_absolute_fix_s=f.time_s;
    if (f.source==AbsoluteSource::Gnss) state_.last_gnss_fix_s=f.time_s;
    else state_.last_non_gnss_fix_s=f.time_s;
    ++state_.accepted_fixes;
    return true;
}

bool ErrorStateIns::correctGroundVelocity(const GroundVelocity& v) {
    if (!state_.initialized || !state_.origin_fixed_and_aligned || !v.valid ||
        !v.ground_referenced || !finite(v.time_s) || !finite(v.velocity_ned_mps) ||
        !finite(v.horizontal_sigma_mps) || !finite(v.vertical_sigma_mps) ||
        v.horizontal_sigma_mps<=0.0 || v.vertical_sigma_mps<=0.0 ||
        std::abs(v.time_s-state_.time_s)>cfg_.max_measurement_age_s) return false;
    return measurementUpdate(v.velocity_ned_mps-state_.velocity_ned_mps,
                             {squared(v.horizontal_sigma_mps),
                              squared(v.horizontal_sigma_mps),
                              squared(v.vertical_sigma_mps)},3,cfg_.max_velocity_nis);
}

bool ErrorStateIns::correctAttitude(const AbsoluteAttitude& a) {
    if(!state_.initialized || !state_.origin_fixed_and_aligned ||
       !a.valid || !a.independently_measured ||
       !finite(a.time_s) || !finite(a.body_to_ned) ||
       !finite(a.angular_sigma_rad) ||
       a.angular_sigma_rad <= 0.0 || a.angular_sigma_rad > 1.0 ||
       std::abs(a.time_s-state_.time_s)>cfg_.max_measurement_age_s)
        return false;
    // Right-invariant attitude residual in the local FRD error coordinates.
    // The nominal orientation is never directly replaced with the observation.
    const Quaternion estimate=state_.body_to_ned;
    const Quaternion conjugate{estimate.w,-estimate.x,-estimate.y,-estimate.z};
    Quaternion difference=normalize(multiply(conjugate,
                                              normalize(a.body_to_ned)));
    if(difference.w<0.0)
        difference={-difference.w,-difference.x,-difference.y,-difference.z};
    const Vec3 imaginary{difference.x,difference.y,difference.z};
    const double sine=norm(imaginary);
    const double angle=2.0*std::atan2(sine,difference.w);
    const Vec3 residual=sine>1.e-12 ?
        imaginary*(angle/sine) : imaginary*2.0;
    const double variance=squared(a.angular_sigma_rad);
    return measurementUpdate(residual,{variance,variance,variance},
                             6,cfg_.max_attitude_nis);
}

double ErrorStateIns::horizontalSigmaM(const State& s) {
    const double a=cell(s.covariance,0,0), b=cell(s.covariance,0,1), c=cell(s.covariance,1,1);
    const double lambda=0.5*(a+c+std::sqrt(squared(a-c)+4.0*squared(b)));
    return std::sqrt(std::max(0.0,lambda));
}
double ErrorStateIns::maximumCovarianceAsymmetry(const State& s) {
    double m=0.0;
    for(int i=0;i<N;++i)
        for(int j=0;j<i;++j)
            m=std::max(m,std::abs(cell(s.covariance,i,j)-cell(s.covariance,j,i)));
    return m;
}

Integrity assess(const State& s, double now_s, bool gnss_denied, const IntegrityPolicy& p) {
    if (!s.initialized || !s.origin_fixed_and_aligned) return {false,"INS uninitialized or origin unaligned"};
    if (!finite(now_s) || now_s < s.time_s || now_s-s.time_s > p.max_state_age_s)
        return {false,"INS state expired"};
    if (!finite(s.last_absolute_fix_s) ||
        now_s-s.last_absolute_fix_s > p.max_fix_age_s)
        return {false,"absolute position unavailable or expired"};
    if (gnss_denied &&
        (!finite(s.last_non_gnss_fix_s) ||
         now_s-s.last_non_gnss_fix_s > p.max_independent_fix_age_during_gnss_denial_s))
        return {false,"GNSS denied and independent absolute fix unavailable"};
    const double sigma=ErrorStateIns::horizontalSigmaM(s);
    if (!finite(sigma) || sigma>p.max_horizontal_sigma_m)
        return {false,"horizontal position uncertainty exceeded"};
    return {true,"research navigation estimate available"};
}

NavigationState makePlanarSnapshot(const State& s) {
    NavigationState out{};
    out.initialized=s.initialized && s.origin_fixed_and_aligned;
    out.time_s=s.time_s;
    out.north_m=s.position_ned_m.x; out.east_m=s.position_ned_m.y;
    out.north_mps=s.velocity_ned_mps.x; out.east_mps=s.velocity_ned_mps.y;
    const Quaternion q=s.body_to_ned;
    out.yaw_rad=std::atan2(2.0*(q.w*q.z+q.x*q.y),
                           1.0-2.0*(q.y*q.y+q.z*q.z));
    out.pos_variance_m2=squared(ErrorStateIns::horizontalSigmaM(s));
    out.vel_variance_m2ps2=std::max(cell(s.covariance,3,3),cell(s.covariance,4,4));
    out.yaw_variance_rad2=cell(s.covariance,8,8);
    out.last_absolute_fix_s=s.last_absolute_fix_s;
    out.last_fix_was_gnss=s.last_gnss_fix_s>=s.last_non_gnss_fix_s;
    return out;
}
} // namespace boat::ins
