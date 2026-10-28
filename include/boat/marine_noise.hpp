#pragma once

#include "boat/marine_identification.hpp"

#include <array>
#include <cstddef>
#include <cstdint>

namespace boat::marine {

struct NoiseTrialSettings {
    double speed_std_mps{};
    double yaw_rate_std_radps{};
    double heading_std_rad{};
    std::size_t trials{60};
    std::uint64_t seed{20261028};
};

struct EstimateDistribution {
    double truth{};
    double mean{};
    double relative_bias{};
    double p05{};
    double p95{};
};

struct NoiseTrialSummary {
    std::size_t trials{};
    std::size_t accepted{};
    std::size_t rejected{};
    // Order: mass, longitudinal linear drag, yaw inertia, yaw linear drag.
    std::array<EstimateDistribution,4> estimates{};
};

// Repeated synthetic identification with white measurement noise at each
// sampled state (a shared noisy state for adjacent record pairs). The range
// [p05,p95] is an empirical percentile interval, NOT a real-data confidence
// interval: dynamics, physical forces and noise law are assumed exact.
// window_samples=1 retains v0.19 finite-difference regression. A larger
// value uses non-overlapping integrated observations (v0.20).
NoiseTrialSummary evaluateMarineNoise(NoiseTrialSettings settings,
                                     std::size_t window_samples = 1);

} // namespace boat::marine
