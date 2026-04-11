---
title: 'Yaw-pitch-roll Wrist'
description: 'Trimmed from the 5-star tool-wrist record; shows nested yaw, pitch, and roll articulations.'
tags:
  - cadquery
  - examples
  - articulation
  - yaw
  - pitch
  - roll
---
# Yaw-pitch-roll Wrist

This excerpt keeps the actual nested-axis layout from the 5-star wrist record: base shell, yaw collar, pitch yoke, and a separate roll spindle.

```python
from sdk import (
    ArticulatedObject,
    ArticulationType,
    MotionLimits,
    Origin,
    mesh_from_cadquery,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tool_wrist")

    base_paint = model.material("base_paint", color=(0.22, 0.22, 0.24, 1.0))
    collar_paint = model.material("collar_paint", color=(0.16, 0.17, 0.19, 1.0))
    yoke_finish = model.material("yoke_finish", color=(0.68, 0.71, 0.74, 1.0))
    spindle_finish = model.material("spindle_finish", color=(0.80, 0.82, 0.84, 1.0))

    base = model.part("base")
    base.visual(mesh_from_cadquery(_make_base_shape(), "tool_wrist_base"), material=base_paint, name="base_shell")

    yaw_collar = model.part("yaw_collar")
    yaw_collar.visual(mesh_from_cadquery(_make_yaw_collar_shape(), "tool_wrist_yaw_collar"), material=collar_paint, name="yaw_collar_shell")

    pitch_yoke = model.part("pitch_yoke")
    pitch_yoke.visual(mesh_from_cadquery(_make_pitch_yoke_shape(), "tool_wrist_pitch_yoke"), material=yoke_finish, name="pitch_yoke_shell")

    roll_spindle = model.part("roll_spindle")
    roll_spindle.visual(mesh_from_cadquery(_make_roll_spindle_shape(), "tool_wrist_roll_spindle"), material=spindle_finish, name="roll_spindle_shell")

    model.articulation("base_to_yaw", ArticulationType.REVOLUTE, parent=base, child=yaw_collar, origin=Origin(xyz=(0.0, 0.0, 0.040)), axis=(0.0, 0.0, 1.0), motion_limits=MotionLimits(effort=18.0, velocity=2.0, lower=-2.4, upper=2.4))
    model.articulation("yaw_to_pitch", ArticulationType.REVOLUTE, parent=yaw_collar, child=pitch_yoke, origin=Origin(xyz=(0.068, 0.0, 0.022)), axis=(0.0, -1.0, 0.0), motion_limits=MotionLimits(effort=12.0, velocity=2.3, lower=-1.1, upper=1.1))
    model.articulation("pitch_to_roll", ArticulationType.REVOLUTE, parent=pitch_yoke, child=roll_spindle, origin=Origin(xyz=(0.060, 0.0, 0.0)), axis=(1.0, 0.0, 0.0), motion_limits=MotionLimits(effort=9.0, velocity=4.0, lower=-3.0, upper=3.0))

    return model
```
