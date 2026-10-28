#pragma once

#include "boat/marine_identification.hpp"

#include <cstddef>

namespace boat::marine {

struct WindowedIdentificationResult {
    MarineParameters parameters;
    std::size_t windows{};
    std::size_t used_samples{};
    double surge_excitation{};
    double yaw_excitation{};
    // Residuals of the integrated, rather than differentiated, equations.
    double surge_integral_rmse_mps{};
    double yaw_integral_rmse_radps{};
};

// Non-overlapping consecutive integration windows reduce differentiation of
// noisy instantaneous velocities. They do not eliminate errors-in-variables.
// Each sample must have a calibrated force and a water-relative body velocity.
WindowedIdentificationResult identifyMarineParametersWindowed(
    const std::vector<IdentificationSample>& samples,
    std::size_t window_samples = 25,
    MarineParameters known = {});

} // namespace boat::marine
