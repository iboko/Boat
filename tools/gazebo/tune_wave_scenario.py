#!/usr/bin/env python3
"""Configure a clearly labelled mild-waves SITL scenario, not real sea state."""
import argparse
from pathlib import Path
import xml.etree.ElementTree as ET

def tune(path: Path, wind_speed: float, steepness: float):
    if not (0.1 <= wind_speed <= 7.0 and 0.05 <= steepness <= 2.0):
        raise ValueError("unsafe or invalid wave configuration")
    tree=ET.parse(path)
    root=tree.getroot()
    wave_nodes = root.findall(".//plugin/wave")
    if len(wave_nodes)<2:
        raise ValueError("expected both Gazebo physical and visual wave fields")
    for node in wave_nodes:
        if node.findtext("algorithm") != "fft":
            raise ValueError("unexpected upstream wave algorithm")
        wind=node.find("wind_speed")
        steep=node.find("steepness")
        if wind is None or steep is None:
            raise ValueError("missing wave amplitude parameters")
        wind.text=f"{wind_speed:.3f}"
        steep.text=f"{steepness:.3f}"
    tree.write(path,encoding="utf-8",xml_declaration=True)
    return len(wave_nodes)

if __name__=="__main__":
    p=argparse.ArgumentParser()
    p.add_argument("--model",type=Path,required=True)
    p.add_argument("--wind-speed",type=float,default=1.0)
    p.add_argument("--steepness",type=float,default=0.4)
    args=p.parse_args()
    count=tune(args.model,args.wind_speed,args.steepness)
    print(f"GAZEBO_MILD_WAVE_BASELINE adjusted={count} wind_speed={args.wind_speed} steepness={args.steepness} (SITL only)")
