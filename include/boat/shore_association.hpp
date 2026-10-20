#pragma once
#include "boat/shore_ranges.hpp"
#include <array>

namespace boat::fixture {

// Deliberately unlabelled radar echoes. No landmark identifiers or
// geometry-based bearing measurements arrive in this packet.
struct AnonymousRanges {
    double time_s{};
    std::array<double,8> ranges_m{};
    unsigned count{};
    bool valid{false};
};
struct AssociatedRanges {
    ShoreRanges ranges{};
    unsigned matched{};
    unsigned unmatched{};
    bool valid{false};
};

// Local, prior-aided map association, NOT radar scan matching.
// A trusted approximate prior is essential; ambiguous or distant returns
// must never be converted into an apparently georeferenced fix.
class ShoreMapAssociator {
public:
    static AnonymousRanges simulateEchoes(ins::Vec3 physics_ned,double t);
    static AssociatedRanges associate(const AnonymousRanges& echoes,
                                      ins::Vec3 trusted_prior_ned,
                                      double gate_m=2.5);
};
} // namespace boat::fixture
