#pragma once

#include "boat/marine_dynamics.hpp"

#include <string>
#include <vector>

namespace boat::marine {

// Deterministic local finite-difference study of the UNCALIBRATED 3-DOF model.
// Each trial repeats the same fixed thrust history and 20 s of zero thrust.
// This is not a physical identification of BlueBoat coefficients.
struct SensitivityRow {
    std::string environment;
    std::string parameter;
    double perturbation_fraction{};
    double baseline_drift_m{};
    double low_drift_m{};
    double high_drift_m{};
    // (drift at 1+h minus drift at 1-h)/(2h), metres per fractional change.
    double central_sensitivity_m{};
    double baseline_yaw_change_rad{};
};

std::vector<SensitivityRow> evaluateSensitivity(double perturbation_fraction=0.20);

} // namespace boat::marine
