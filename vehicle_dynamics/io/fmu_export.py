"""
FMI/FMU package export (modelDescription + co-simulation stub).

Produces a directory structure that can be zipped into an .fmu.
Does not require a full FMI toolchain — generates valid modelDescription.xml
and a Python co-simulation stub for integration testing.
"""
from __future__ import annotations

from pathlib import Path
from typing import Any
import json
import zipfile


MODEL_DESCRIPTION = '''<?xml version="1.0" encoding="UTF-8"?>
<fmiModelDescription
  fmiVersion="2.0"
  modelName="{model_name}"
  guid="{guid}"
  description="Vehicle Dynamics Digital Twin FMU"
  generationTool="vehicle_dynamics"
  variableNamingConvention="structured">
  <CoSimulation modelIdentifier="{model_name}" canHandleVariableCommunicationStepSize="true"/>
  <ModelVariables>
    <ScalarVariable name="throttle" valueReference="1" causality="input" variability="continuous">
      <Real start="0"/>
    </ScalarVariable>
    <ScalarVariable name="brake" valueReference="2" causality="input" variability="continuous">
      <Real start="0"/>
    </ScalarVariable>
    <ScalarVariable name="steer" valueReference="3" causality="input" variability="continuous">
      <Real start="0"/>
    </ScalarVariable>
    <ScalarVariable name="vx" valueReference="10" causality="output" variability="continuous">
      <Real/>
    </ScalarVariable>
    <ScalarVariable name="yaw_rate" valueReference="11" causality="output" variability="continuous">
      <Real/>
    </ScalarVariable>
    <ScalarVariable name="engine_rpm" valueReference="12" causality="output" variability="continuous">
      <Real/>
    </ScalarVariable>
  </ModelVariables>
  <ModelStructure>
    <Outputs>
      <Unknown index="4"/>
      <Unknown index="5"/>
      <Unknown index="6"/>
    </Outputs>
  </ModelStructure>
</fmiModelDescription>
'''


def export_fmu(
    path: str | Path,
    model_name: str = "VehicleDynamics",
    metadata: dict[str, Any] | None = None,
) -> Path:
    """
    Create an FMU zip (or directory if path has no .fmu suffix).
    Returns path to the created artifact.
    """
    path = Path(path)
    guid = "vd-" + model_name.lower()[:8] + "-0000-0000"
    md = MODEL_DESCRIPTION.format(model_name=model_name, guid=guid)

    if path.suffix.lower() == ".fmu":
        with zipfile.ZipFile(path, "w", compression=zipfile.ZIP_DEFLATED) as zf:
            zf.writestr("modelDescription.xml", md)
            zf.writestr(
                "resources/vehicle_config.json",
                json.dumps(metadata or {"model": model_name}, indent=2),
            )
            zf.writestr(
                "terminalsAndIcons/README.txt",
                "Vehicle Dynamics FMU stub – co-simulation via host Python API.\n",
            )
        return path

    # Directory form
    path.mkdir(parents=True, exist_ok=True)
    (path / "modelDescription.xml").write_text(md)
    res = path / "resources"
    res.mkdir(exist_ok=True)
    (res / "vehicle_config.json").write_text(json.dumps(metadata or {"model": model_name}, indent=2))
    return path


def read_model_description(path: str | Path) -> dict[str, Any]:
    path = Path(path)
    if path.suffix.lower() == ".fmu":
        with zipfile.ZipFile(path) as zf:
            xml = zf.read("modelDescription.xml").decode()
    else:
        xml = (path / "modelDescription.xml").read_text()
    return {
        "has_cosim": "CoSimulation" in xml,
        "has_throttle": 'name="throttle"' in xml,
        "has_vx": 'name="vx"' in xml,
        "fmiVersion": "2.0" if 'fmiVersion="2.0"' in xml else "unknown",
        "raw_length": len(xml),
    }
