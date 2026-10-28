#pragma once

#include "boat/marine_dynamics.hpp"

#include <cstddef>
#include <vector>

namespace boat::marine {

// A pair of consecutive *measured* water-relative body velocities and yaw
// rates, with calibrated forces/moments expressed in N and N*m. Positions
// alone, PWM commands, and ground speed do not satisfy this contract.
struct IdentificationSample {
    MarineState before;
    MarineState after;
    MarineControl applied;
    MarineEnvironment environment;
};

struct IdentificationResult {
    MarineParameters parameters;
    std::size_t sample_count{};
    // Normalized 2x2 regression determinants: 0 means rank deficiency.
    // These are excitation indicators, NOT statistical confidence levels.
    double surge_excitation{};
    double yaw_excitation{};
    double surge_residual_rmse_mps2{};
    double yaw_residual_rmse_radps2{};
};

// Separately estimate mass/surge drag and yaw inertia/yaw drag by solving the
// discrete equations actually used by MarineThreeDof. All other coefficients
// and the environment must be known. Throws on invalid measurements, singular
// excitation, or nonphysical estimates. No uncertainty interval is implied.
IdentificationResult identifyMarineParameters(
    const std::vector<IdentificationSample>& samples,
    MarineParameters known = {});

} // namespace boat::marine
