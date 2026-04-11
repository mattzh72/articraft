---
title: 'Single Roll Axis Module'
description: 'Trimmed from the 5-star motorized roll-stage record; shows the framed base, sensor tube, and coaxial roll articulation.'
tags:
  - cadquery
  - examples
  - articulation
  - revolute
  - roll
  - sensor
---
# Single Roll Axis Module

The 5-star roll example is more than a cylinder between two plates: it carries a real frame, motor body, and sensor-tube assembly around the roll axis.

```python
from math import pi

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    mesh_from_cadquery,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="motorized_roll_stage", assets=ASSETS)

    model.material("machined_aluminum", rgba=(0.76, 0.78, 0.81, 1.0))
    model.material("motor_black", rgba=(0.18, 0.19, 0.21, 1.0))
    model.material("tube_black", rgba=(0.12, 0.13, 0.15, 1.0))
    model.material("signal_amber", rgba=(0.87, 0.52, 0.12, 1.0))

    base = model.part("base")
    base.visual(
        mesh_from_cadquery(_build_base_frame_shape(), "roll_stage_base_frame.obj", assets=ASSETS),
        material="machined_aluminum",
    )
    base.visual(
        mesh_from_cadquery(_build_motor_shape(), "roll_stage_motor.obj", assets=ASSETS),
        material="motor_black",
    )
    base.inertial = Inertial.from_geometry(
        Box((BASE_LEN, BASE_WIDTH, SUPPORT_TOP_Z)),
        mass=1.6,
        origin=Origin(xyz=(0.0, 0.0, SUPPORT_TOP_Z / 2.0)),
    )

    sensor_tube = model.part("sensor_tube")
    sensor_tube.visual(
        mesh_from_cadquery(_build_sensor_body_shape(), "sensor_tube_body.obj", assets=ASSETS),
        material="tube_black",
    )
    sensor_tube.visual(
        mesh_from_cadquery(_build_sensor_flag_shape(), "sensor_tube_flag.obj", assets=ASSETS),
        material="signal_amber",
    )
    sensor_tube.inertial = Inertial.from_geometry(
        Cylinder(radius=TUBE_RADIUS, length=0.15),
        mass=0.42,
        origin=Origin(xyz=(0.075, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
    )

    model.articulation(
        "tube_roll",
        ArticulationType.REVOLUTE,
        parent=base,
        child=sensor_tube,
        origin=Origin(xyz=(-SUPPORT_SPAN / 2.0, 0.0, AXIS_HEIGHT)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=-1.2, upper=1.2, effort=6.0, velocity=2.5),
    )

    return model
```
