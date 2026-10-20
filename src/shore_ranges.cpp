#include "boat/shore_ranges.hpp"
#include <algorithm>
#include <cmath>
#include <limits>

namespace boat::fixture {
namespace {
constexpr double kMinRangeM=5.0;
constexpr double kMaxRangeM=350.0;
constexpr double kInlierThresholdM=1.2;
constexpr double kTiny=1e-9;
struct P2 { double n{},e{}; };
double squared(double x){return x*x;}
double distance(P2 a,P2 b){return std::hypot(a.n-b.n,a.e-b.e);}
bool finite(double a){return std::isfinite(a);}
bool solveTwo(double a,double b,double c,double d,double y1,double y2,P2& out) {
    const double det=a*d-b*c;
    if(!finite(det)||std::abs(det)<kTiny)return false;
    out={(y1*d-b*y2)/det,(a*y2-y1*c)/det};
    return finite(out.n)&&finite(out.e);
}
P2 point(const ShoreReflector& r){return {r.north_m,r.east_m};}
P2 point(ins::Vec3 v){return {v.x,v.y};}
}

const std::array<ShoreReflector,ShoreRangeModel::kCount>&
ShoreRangeModel::surveyedReflectors() {
    static const std::array<ShoreReflector,kCount> refs{{
        {0,-120.0,-50.0},
        {1,-90.0,110.0},
        {2,90.0,-120.0},
        {3,130.0,90.0},
        {4,10.0,160.0},
    }};
    return refs;
}

ShoreRanges ShoreRangeModel::measure(ins::Vec3 truth,double t,bool outlier) {
    ShoreRanges ranges{};
    if(!finite(t)||!finite(truth.x)||!finite(truth.y))return ranges;
    ranges.time_s=t;
    for(const auto& landmark:surveyedReflectors()){
        const double raw=std::hypot(truth.x-landmark.north_m,
                                    truth.y-landmark.east_m);
        if(raw<kMinRangeM || raw>kMaxRangeM)continue;
        // Artificial ranging error only. Not radar cross-section, propagation,
        // multipath, bearing association or line-of-sight obstruction.
        double observation=raw+0.11*std::sin(t*0.71+landmark.id*1.93);
        if(outlier && t>=3.0 && t<5.0 && landmark.id==2)
            observation+=12.0;
        ranges.readings[ranges.count++]={landmark.id,observation};
    }
    ranges.valid=ranges.count>=3;
    return ranges;
}

ShoreSolution ShoreRangeModel::solve(const ShoreRanges& ranges,ins::Vec3 prior) {
    ShoreSolution output{};
    if(!ranges.valid || ranges.count<3 ||
       ranges.count>ranges.readings.size() ||
       !finite(prior.x)||!finite(prior.y)||!finite(prior.z)) return output;
    const auto& landmarks=surveyedReflectors();
    std::array<P2,8> sites{};
    std::array<double,8> distances{};
    for(unsigned k=0;k<ranges.count;++k){
        const auto item=ranges.readings[k];
        if(item.reflector_id<0 ||
           item.reflector_id>=static_cast<int>(landmarks.size()) ||
           !finite(item.distance_m) || item.distance_m<kMinRangeM ||
           item.distance_m>kMaxRangeM+15.0) return output;
        for(unsigned j=0;j<k;++j)
            if(ranges.readings[j].reflector_id==item.reflector_id)
                return output; // duplicate same reflector cannot count as independent
        sites[k]=point(landmarks[static_cast<std::size_t>(item.reflector_id)]);
        distances[k]=item.distance_m;
    }
    unsigned best_inliers=0;
    double best_cost=std::numeric_limits<double>::infinity();
    P2 best=point(prior);
    for(unsigned i=0;i<ranges.count;++i)
        for(unsigned j=i+1;j<ranges.count;++j)
            for(unsigned k=j+1;k<ranges.count;++k) {
                const auto& base=sites[i];
                const auto& q=sites[j];
                const auto& r=sites[k];
                P2 candidate{};
                if(!solveTwo(
                    2.0*(q.n-base.n),2.0*(q.e-base.e),
                    2.0*(r.n-base.n),2.0*(r.e-base.e),
                    squared(q.n)+squared(q.e)-squared(base.n)-squared(base.e)+
                        squared(distances[i])-squared(distances[j]),
                    squared(r.n)+squared(r.e)-squared(base.n)-squared(base.e)+
                        squared(distances[i])-squared(distances[k]),
                    candidate)) continue;
                unsigned count=0;
                double error=0;
                for(unsigned n=0;n<ranges.count;++n) {
                    const double deviation=distance(candidate,sites[n])-distances[n];
                    if(std::abs(deviation)<=kInlierThresholdM) {
                        ++count;
                        error+=deviation*deviation;
                    }
                }
                if(count>best_inliers ||
                   (count==best_inliers && error<best_cost)) {
                    best_inliers=count;
                    best_cost=error;
                    best=candidate;
                }
            }
    if(best_inliers<3)return output;
    // Refine only consensus observations, re-selecting inliers each iteration.
    for(int step=0;step<8;++step) {
        double h00=0,h01=0,h11=0,b0=0,b1=0;
        unsigned active=0;
        for(unsigned k=0;k<ranges.count;++k) {
            const double predicted=distance(best,sites[k]);
            const double residual=distances[k]-predicted;
            if(predicted<kTiny||std::abs(residual)>kInlierThresholdM)continue;
            const double j0=(best.n-sites[k].n)/predicted;
            const double j1=(best.e-sites[k].e)/predicted;
            h00+=j0*j0;h01+=j0*j1;h11+=j1*j1;
            b0+=j0*residual;b1+=j1*residual;
            ++active;
        }
        if(active<3)return output;
        P2 correction{};
        if(!solveTwo(h00,h01,h01,h11,b0,b1,correction))return output;
        best.n+=correction.n;best.e+=correction.e;
        if(std::hypot(correction.n,correction.e)<1e-7)break;
    }
    unsigned accepted=0;
    double sum2=0;
    for(unsigned k=0;k<ranges.count;++k) {
        const double residual=distance(best,sites[k])-distances[k];
        if(std::abs(residual)<=kInlierThresholdM) {
            sum2+=residual*residual;
            ++accepted;
        }
    }
    if(accepted<3 || !finite(best.n)||!finite(best.e))return output;
    output.valid=true;
    output.position_ned_m={best.n,best.e,prior.z};
    output.inliers=accepted;
    output.residual_rms_m=std::sqrt(sum2/accepted);
    return output;
}
} // namespace boat::fixture
