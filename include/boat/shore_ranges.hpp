#pragma once
#include "boat/ins.hpp"
#include <array>
#include <cstddef>

namespace boat::fixture {

// Identified, surveyed shoreline reflectors in a fixed local NED datum.
// Research model: each reflector has a known ID; no radar scan/association.
struct ShoreReflector {
    int id{};
    double north_m{};
    double east_m{};
};
struct RangeReading {
    int reflector_id{-1};
    double distance_m{};
};
struct ShoreRanges {
    double time_s{};
    std::array<RangeReading,8> readings{};
    unsigned count{};
    bool valid{false};
};
struct ShoreSolution {
    bool valid{false};
    ins::Vec3 position_ned_m{};
    unsigned inliers{};
    double residual_rms_m{};
};

class ShoreRangeModel {
public:
    static constexpr unsigned kCount=5;
    static const std::array<ShoreReflector,kCount>& surveyedReflectors();

    // Raw ranges are synthesized from SITL physical truth; a single deliberate
    // outlier tests landmark innovation rejection. No coordinate fix is passed.
    static ShoreRanges measure(ins::Vec3 physics_position_ned_m,
                               double time_s, bool inject_outlier=true);

    // Geometry-only range multilateration: triples for outlier consensus,
    // followed by robust least squares refinement. No physics truth input.
    static ShoreSolution solve(const ShoreRanges& measurement,
                               ins::Vec3 prior_position_ned_m);
};

} // namespace boat::fixture
