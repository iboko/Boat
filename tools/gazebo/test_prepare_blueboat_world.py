#!/usr/bin/env python3
import tempfile
from pathlib import Path
from xml.etree import ElementTree as ET
from prepare_blueboat_world import inject_blueboat

with tempfile.TemporaryDirectory() as tmp:
    root=Path(tmp)
    source=root/"waves.sdf"
    output=root/"blueboat_waves.sdf"
    source.write_text(
        '<?xml version="1.0"?><sdf version="1.9"><world name="waves">'
        '<plugin filename="fake-test.so" name="test"/>'
        '<include><uri>model://waves</uri></include></world></sdf>',
        encoding="utf-8")
    inject_blueboat(source,output)
    inject_blueboat(output,output)
    node=ET.parse(output).getroot()
    boats=[i for i in node.findall("world/include")
           if i.findtext("uri")=="model://blueboat"]
    assert len(boats)==1
    assert boats[0].find("pose").get("degrees")=="true"
    assert node.find("world/plugin").get("filename")=="fake-test.so"
    bad=root/"bad.sdf"
    bad.write_text('<sdf version="1.9"><world name="wrong"/></sdf>')
    try:
        inject_blueboat(bad,output)
        raise AssertionError("missing upstream water was accepted")
    except ValueError:
        pass
print("PASS: BlueBoat wave SDF preparation, preserved plugins, duplicate-safe")
