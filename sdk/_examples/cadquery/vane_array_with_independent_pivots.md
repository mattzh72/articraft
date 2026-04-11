---
title: 'Vane Array with Independent Pivots'
description: 'Trimmed from the 5-star vane-array record; shows a rigid frame with repeated independent blade parts.'
tags:
  - cadquery
  - examples
  - articulation
  - revolute
  - array
  - vane
---
# Vane Array with Independent Pivots

The 5-star vane-array record is useful because it shows how to author repeated independent articulations against a single grounded frame.

```python
from sdk import ArticulatedObject, ArticulationType, Box, Inertial, MotionLimits, Origin, mesh_from_cadquery


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="vane_array", assets=ASSETS)

    model.material("frame_finish", rgba=(0.19, 0.21, 0.24, 1.0))
    model.material("blade_finish", rgba=(0.78, 0.80, 0.83, 1.0))

    frame_mesh = mesh_from_cadquery(_make_frame_shape(), "frame.obj", assets=ASSETS)
    blade_mesh = mesh_from_cadquery(_make_blade_shape(), "blade.obj", assets=ASSETS)

    frame = model.part("frame")
    frame.visual(frame_mesh, material="frame_finish")
    frame.inertial = Inertial.from_geometry(Box((FRAME_OUTER_WIDTH, FRAME_DEPTH, FRAME_OUTER_HEIGHT)), mass=2.4)

    for index, (blade_name, z_offset) in enumerate(zip(BLADE_NAMES, BLADE_Z_OFFSETS)):
        blade = model.part(blade_name)
        blade.visual(blade_mesh, material="blade_finish")
        blade.inertial = Inertial.from_geometry(Box((BLADE_COLLISION_LENGTH, BLADE_COLLISION_CHORD, BLADE_COLLISION_THICKNESS)), mass=0.14, origin=Origin(xyz=(BLADE_COLLISION_ORIGIN_X, BLADE_Y_OFFSET, 0.0)))
        model.articulation(
            f"frame_to_blade_{index + 1}",
            ArticulationType.REVOLUTE,
            parent=frame,
            child=blade,
            origin=Origin(xyz=(LEFT_PIVOT_X, 0.0, z_offset)),
            axis=(1.0, 0.0, 0.0),
            motion_limits=MotionLimits(lower=-BLADE_LIMIT, upper=BLADE_LIMIT, effort=1.0, velocity=2.5),
        )

    return model
```
