#include "boat/shore_association.hpp"
#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>

namespace boat::fixture {
AnonymousRanges ShoreMapAssociator::simulateEchoes(ins::Vec3 truth,double t) {
    AnonymousRanges output{};
    const auto labeled=ShoreRangeModel::measure(truth,t,true);
    if(!labeled.valid)return output;
    output.time_s=t;
    output.count=labeled.count;
    // Deliberately permute observation order; it must carry NO map IDs.
    const unsigned shift=static_cast<unsigned>(std::floor(std::abs(t)*3.0))%labeled.count;
    for(unsigned i=0;i<labeled.count;++i)
        output.ranges_m[i]=labeled.readings[(i+shift)%labeled.count].distance_m;
    output.valid=true;
    return output;
}

AssociatedRanges ShoreMapAssociator::associate(
    const AnonymousRanges& observed,ins::Vec3 prior,double gate) {
    AssociatedRanges result{};
    if(!observed.valid || observed.count<3 ||
       observed.count>observed.ranges_m.size() ||
       !std::isfinite(observed.time_s) ||
       !std::isfinite(prior.x)||!std::isfinite(prior.y) ||
       !std::isfinite(prior.z)||!std::isfinite(gate) ||
       gate<0.3||gate>5.0)return result;
    const auto& map=ShoreRangeModel::surveyedReflectors();
    std::array<double,8> predicted{};
    for(unsigned j=0;j<map.size();++j)
        predicted[j]=std::hypot(prior.x-map[j].north_m,prior.y-map[j].east_m);
    for(unsigned i=0;i<observed.count;++i)
        if(!std::isfinite(observed.ranges_m[i]) ||
           observed.ranges_m[i]<5.0 || observed.ranges_m[i]>365.0)
            return result;
    std::array<int,8> assignment{},best{};
    assignment.fill(-1);
    best.fill(-1);
    unsigned best_count=0;
    double best_cost=std::numeric_limits<double>::infinity();

    // Global one-to-one optimal correspondence of unordered observations.
    // Unmatched returns are permitted, but >=3 unique matches are required.
    std::function<void(unsigned,unsigned,unsigned,double)> search =
    [&](unsigned i,unsigned used,unsigned matched,double cost) {
        if(matched+(observed.count-i)<best_count)return;
        if(i==observed.count) {
            if(matched>best_count || (matched==best_count && cost<best_cost)) {
                best_count=matched;best_cost=cost;best=assignment;
            }
            return;
        }
        assignment[i]=-1;
        search(i+1,used,matched,cost);
        for(unsigned j=0;j<map.size();++j) {
            if(used&(1U<<j))continue;
            const double residual=observed.ranges_m[i]-predicted[j];
            if(std::abs(residual)>gate)continue;
            assignment[i]=static_cast<int>(j);
            search(i+1,used|(1U<<j),matched+1,
                   cost+residual*residual);
        }
        assignment[i]=-1;
    };
    search(0,0,0,0);
    if(best_count<3)return result;
    result.ranges.time_s=observed.time_s;
    for(unsigned i=0;i<observed.count;++i) {
        if(best[i]<0)continue;
        result.ranges.readings[result.ranges.count++]={
            best[i],observed.ranges_m[i]};
    }
    result.ranges.valid=true;
    result.matched=best_count;
    result.unmatched=observed.count-best_count;
    result.valid=true;
    return result;
}
} // namespace boat::fixture
