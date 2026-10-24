#!/usr/bin/env python3
"""Audit that synthetic C++ marine logs cannot masquerade as validated data."""
from run_onboard_environment_matrix import parse_result


sample=("PASS_MARINE_ROUTE navigation=onboard environment=nominal "
        "waypoints=3 drift_after_neutral_20s_m=5.175 eskf_p95_m=0.222 "
        "max_eskf_error_m=0.253 stopped_for_integrity=0 "
        "current_n=0.350 current_e=-0.200 wind_n=2.000 wind_e=3.000 "
        "model=unvalidated_3dof_surrogate NOT_GAZEBO=true\n")
result=parse_result(sample,"nominal","onboard-route")
assert result["waypoints"]==3 and result["stopped_for_integrity"]==0

for forged in (sample.replace("navigation=onboard","navigation=shore"),
               sample.replace("NOT_GAZEBO=true","NOT_GAZEBO=false"),
               sample.replace("waypoints=3","waypoints=2"),
               sample.replace("eskf_p95_m=0.222","eskf_p95_m=nan"),
               sample.replace("stopped_for_integrity=0","stopped_for_integrity=1")):
    try:
        parse_result(forged,"nominal","onboard-route")
        raise AssertionError("invalid marine trial result accepted")
    except ValueError:
        pass

blackout=(sample.replace("PASS_MARINE_ROUTE","PASS_MARINE_BLACKOUT")
          .replace("waypoints=3","waypoints=0")
          .replace("stopped_for_integrity=0","stopped_for_integrity=1"))
assert parse_result(blackout,"nominal","onboard-blackout")["stopped_for_integrity"]==1
print("PASS: onboard environmental matrix log rejects false sensor, mode, "
      "physics source and navigation outcomes")
