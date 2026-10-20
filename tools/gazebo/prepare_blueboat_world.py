#!/usr/bin/env python3
"""Prepare an upstream Wave Sim world for BlueBoat; does NOT run Gazebo.

Upstream references:
ArduPilot/SITL_Models/Gazebo/docs/BlueBoat.md
srmainwaring/asv_wave_sim/gz-waves-models/worlds/waves.sdf
"""
import argparse
from pathlib import Path
import xml.etree.ElementTree as ET


def inject_blueboat(source: Path, destination: Path) -> None:
    root = ET.parse(source).getroot()
    if root.tag != "sdf":
        raise ValueError("Expected SDF root")
    world = root.find("world")
    if world is None:
        raise ValueError("No Gazebo world")
    has_waves = any(
        include.findtext("uri") == "model://waves"
        for include in world.findall("include")
    )
    if not has_waves:
        raise ValueError("Missing upstream Wave Sim model://waves include")
    boats = [include for include in world.findall("include")
             if include.findtext("uri") == "model://blueboat"]
    if len(boats) > 1:
        raise ValueError("Duplicate BlueBoat instances")
    if not boats:
        element = ET.SubElement(world, "include")
        ET.SubElement(element, "pose", {"degrees": "true"}).text = (
            "0 0 0 0 0 90"
        )
        ET.SubElement(element, "uri").text = "model://blueboat"
    destination.parent.mkdir(parents=True, exist_ok=True)
    ET.ElementTree(root).write(destination, encoding="utf-8",
                               xml_declaration=True)
    checked = ET.parse(destination).getroot()
    assert sum(node.findtext("uri") == "model://blueboat"
               for node in checked.findall("world/include")) == 1


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--upstream-world", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    inject_blueboat(args.upstream_world, args.output)
    print("BLUEBOAT_WORLD_PREPARED (not Gazebo execution):", args.output)
