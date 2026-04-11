---
title: 'Orthogonal XY Stage'
description: 'Trimmed from the 5-star precision XY stage record; shows stacked X and Y carriages with separate rail sets.'
tags:
  - cadquery
  - examples
  - articulation
  - prismatic
  - xy
  - cartesian
---
# Orthogonal XY Stage

This excerpt keeps the real split from the 5-star XY stage record: grounded base, X carriage with its own rails, and a separate Y stage on top.

```python
from sdk import ArticulatedObject, ArticulationType, Box, Inertial, MotionLimits, Origin


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="precision_xy_stage", assets=ASSETS)

    model.material("anodized_black", rgba=(0.16, 0.17, 0.19, 1.0))
    model.material("ground_steel", rgba=(0.72, 0.74, 0.78, 1.0))
    model.material("machined_gray", rgba=(0.60, 0.62, 0.66, 1.0))
    model.material("plate_blue", rgba=(0.41, 0.47, 0.60, 1.0))

    base = model.part("base")
    _add_mesh_visual(base, _base_body_shape(), OBJ_FILENAMES[0], "anodized_black")
    _add_mesh_visual(base, _x_rails_shape(), OBJ_FILENAMES[1], "ground_steel")
    base.inertial = Inertial.from_geometry(Box((BASE_L, BASE_W, BASE_T)), mass=4.2)

    x_stage = model.part("x_stage")
    _add_mesh_visual(x_stage, _x_carriage_shape(), OBJ_FILENAMES[2], "machined_gray")
    _add_mesh_visual(x_stage, _y_rails_shape(), OBJ_FILENAMES[3], "ground_steel")

    y_stage = model.part("y_stage")
    _add_mesh_visual(y_stage, _y_stage_shape(), OBJ_FILENAMES[4], "plate_blue")

    model.articulation(
        "base_to_x",
        ArticulationType.PRISMATIC,
        parent=base,
        child=x_stage,
        origin=Origin(xyz=(0.0, 0.0, X_JOINT_Z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(lower=-0.060, upper=0.060, effort=120.0, velocity=0.25),
    )
    model.articulation(
        "x_to_y",
        ArticulationType.PRISMATIC,
        parent=x_stage,
        child=y_stage,
        origin=Origin(xyz=(0.0, 0.0, Y_JOINT_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(lower=-0.035, upper=0.035, effort=90.0, velocity=0.25),
    )

    return model
```
