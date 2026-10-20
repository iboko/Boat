#include "boat/ins.hpp"

#include <cmath>
#include <iomanip>
#include <iostream>

int main() {
    using namespace boat::ins;
    constexpr double gravity=9.80665;
    constexpr double dt=0.01;
    ErrorStateIns estimator;
    estimator.initialize(0,{0,0,0},{1.5,0,0},{1,0,0,0},true);
    // Synthetic absolute fix initializes integrity. Not a real sensor input.
    estimator.correctPosition({0,{0,0,0},0.8,1.5,AbsoluteSource::Gnss,true,false});
    std::cout << "time_s,true_n_m,estimated_n_m,estimated_e_m,"
                 "error_m,sigma_horizontal_m,gnss_denied,independent_fix,integrity_ok\n";
    double max_error=0;
    unsigned accepted_shore=0;
    for(int k=1;k<=20000;++k) {
        const double t=k*dt;
        // Stationary gravity projection, constant-speed vessel.
        // Deterministic gyro/accel bias causes dead-reckoning drift.
        if(!estimator.propagate({t,{0.005,0.0,-gravity+0.002},
                                {0.0,0.0,0.0003},true}))return 2;
        const bool denied = t>20.0 && t<160.0;
        if(k%100==0) {
            if(!denied) {
                estimator.correctPosition({t,{1.5*t+0.1*std::sin(t),0.15*std::cos(t),0},
                                          0.9,1.5,AbsoluteSource::Gnss,true,false});
            } else {
                // This simulates the OUTPUT of radar/map matching, NOT radar processing.
                if(estimator.correctPosition({t,{1.5*t+0.25*std::sin(t),0.25*std::cos(t),0},
                                             1.5,2.0,AbsoluteSource::ShoreRadar,true,true}))
                    ++accepted_shore;
            }
        }
        if(k%100==0) {
            const auto& s=estimator.state();
            const bool good=assess(s,t,denied).valid;
            const double error=std::hypot(s.position_ned_m.x-1.5*t,s.position_ned_m.y);
            max_error=std::max(max_error,error);
            std::cout << std::fixed << std::setprecision(4)
                      << t << ',' << 1.5*t << ',' << s.position_ned_m.x << ','
                      << s.position_ned_m.y << ',' << error << ','
                      << ErrorStateIns::horizontalSigmaM(s) << ','
                      << int(denied) << ',' << int(std::isfinite(s.last_non_gnss_fix_s))
                      << ',' << int(good) << '\n';
        }
    }
    // Radar is present in the GNSS-denied interval. Any bound here is scenario-specific.
    if(accepted_shore<100 || max_error>20.0)return 3;
    return 0;
}
