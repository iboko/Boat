#include "boat/marine_sensitivity.hpp"

#include <cmath>
#include <iostream>
#include <stdexcept>

namespace {
void check(bool ok,const char* why){if(!ok)throw std::runtime_error(why);}
}

int main(){
    using namespace boat::marine;
    try {
        const auto rows=evaluateSensitivity(0.20);
        const auto repeat=evaluateSensitivity(0.20);
        check(rows.size()==18 && repeat.size()==rows.size(),
              "expected 3 profiles x 6 independent coefficients");
        unsigned responsive=0;
        double calm=0.0,current=0.0,wind=0.0;
        for(std::size_t i=0;i<rows.size();++i) {
            const auto& row=rows[i];
            check(row.environment==repeat[i].environment &&
                  row.parameter==repeat[i].parameter &&
                  row.baseline_drift_m==repeat[i].baseline_drift_m &&
                  row.low_drift_m==repeat[i].low_drift_m &&
                  row.high_drift_m==repeat[i].high_drift_m,
                  "parameter sweep is not reproducible");
            check(std::isfinite(row.baseline_drift_m) &&
                  std::isfinite(row.low_drift_m) &&
                  std::isfinite(row.high_drift_m) &&
                  std::isfinite(row.central_sensitivity_m) &&
                  row.baseline_drift_m>=0.0 && row.low_drift_m>=0.0 &&
                  row.high_drift_m>=0.0,
                  "nonfinite or negative post-neutral displacement");
            const auto expected=(row.high_drift_m-row.low_drift_m)/0.40;
            check(std::abs(expected-row.central_sensitivity_m)<1e-12,
                  "central finite difference has incorrect denominator");
            if(std::abs(row.central_sensitivity_m)>0.05)++responsive;
            if(row.parameter=="mass_kg") {
                if(row.environment=="calm")calm=row.baseline_drift_m;
                if(row.environment=="current")current=row.baseline_drift_m;
                if(row.environment=="wind")wind=row.baseline_drift_m;
            }
        }
        check(responsive>=5,"sensitivity matrix does not excite dynamics");
        check(std::abs(current-calm)>0.1 && std::abs(wind-calm)>0.1,
              "forcing profiles are indistinguishable");
        bool rejected=false;
        try {(void)evaluateSensitivity(0.0);}
        catch(const std::invalid_argument&){rejected=true;}
        check(rejected,"zero perturbation accepted");
        rejected=false;
        try {(void)evaluateSensitivity(0.3);}
        catch(const std::invalid_argument&){rejected=true;}
        check(rejected,"unsafe perturbation fraction accepted");
        std::cout<<"PASS: deterministic 18-case marine sensitivity; "
                    "distinct environmental loads and finite differences\n";
        return 0;
    } catch(const std::exception& e) {
        std::cerr<<"FAIL: "<<e.what()<<'\n';
        return 1;
    }
}
