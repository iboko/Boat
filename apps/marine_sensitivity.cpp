#include "boat/marine_sensitivity.hpp"

#include <iomanip>
#include <iostream>
#include <string>

int main(int argc,char** argv) {
    if(argc!=2 || std::string(argv[1])!="--model-only-sensitivity") {
        std::cerr<<"Usage: boat_marine_sensitivity --model-only-sensitivity\n";
        return 2;
    }
    try {
        const auto rows=boat::marine::evaluateSensitivity();
        std::cout<<"environment,parameter,perturbation_fraction,baseline_drift_m,"
                     "low_drift_m,high_drift_m,central_sensitivity_m,"
                     "baseline_yaw_change_rad\n";
        std::cout<<std::fixed<<std::setprecision(9);
        for(const auto& row:rows)
            std::cout<<row.environment<<','<<row.parameter<<','
                     <<row.perturbation_fraction<<','<<row.baseline_drift_m<<','
                     <<row.low_drift_m<<','<<row.high_drift_m<<','
                     <<row.central_sensitivity_m<<','
                     <<row.baseline_yaw_change_rad<<'\n';
        return 0;
    } catch(const std::exception& e) {
        std::cerr<<"FAIL_MARINE_SENSITIVITY: "<<e.what()<<'\n';
        return 1;
    }
}
