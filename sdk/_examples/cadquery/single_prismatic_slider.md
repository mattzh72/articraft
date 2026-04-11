---
title: 'Single Prismatic Slider'
description: 'Trimmed from the 5-star machine-tool linear stage record; shows the rail, carriage, wipers, and travel limits.'
tags:
  - cadquery
  - examples
  - articulation
  - prismatic
  - slider
  - carriage
---
# Single Prismatic Slider

This is the core `build_object_model()` excerpt from the 5-star linear-stage record. It keeps the real carriage details instead of collapsing the example down to bare boxes.

```python
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Inertial,
    MotionLimits,
    Origin,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="machine_tool_linear_stage", assets=ASSETS)
    model.material("rail_steel", rgba=(0.60, 0.63, 0.68, 1.0))
    model.material("carriage_steel", rgba=(0.72, 0.74, 0.78, 1.0))
    model.material("wiper_black", rgba=(0.08, 0.08, 0.09, 1.0))

    rail = model.part("rail")
    rail.visual(_build_rail_mesh(), material="rail_steel")
    rail.inertial = Inertial.from_geometry(
        Box((RAIL_LENGTH, RAIL_WIDTH, RAIL_HEIGHT)),
        mass=3.2,
        origin=Origin(xyz=(0.0, 0.0, -RAIL_HEIGHT / 2.0)),
    )

    carriage = model.part("carriage")
    carriage.visual(_build_carriage_mesh(), material="carriage_steel")
    carriage.visual(
        Box((WIPER_THICKNESS, CARRIAGE_WIDTH * 0.90, WIPER_HEIGHT)),
        origin=Origin(
            xyz=(CARRIAGE_LENGTH / 2.0 - WIPER_THICKNESS / 2.0, 0.0, WIPER_HEIGHT / 2.0)
        ),
        material="wiper_black",
    )
    carriage.visual(
        Box((WIPER_THICKNESS, CARRIAGE_WIDTH * 0.90, WIPER_HEIGHT)),
        origin=Origin(
            xyz=(-CARRIAGE_LENGTH / 2.0 + WIPER_THICKNESS / 2.0, 0.0, WIPER_HEIGHT / 2.0)
        ),
        material="wiper_black",
    )
    carriage.inertial = Inertial.from_geometry(
        Box((CARRIAGE_LENGTH, CARRIAGE_WIDTH, CARRIAGE_HEIGHT)),
        mass=1.0,
        origin=Origin(xyz=(0.0, 0.0, CARRIAGE_HEIGHT / 2.0)),
    )

    model.articulation(
        "rail_to_carriage",
        ArticulationType.PRISMATIC,
        parent="rail",
        child="carriage",
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=-TRAVEL_HALF_RANGE,
            upper=TRAVEL_HALF_RANGE,
            effort=1500.0,
            velocity=0.4,
        ),
    )

    return model
```
