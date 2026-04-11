---
title: 'XYZ Cartesian Stage'
description: 'Trimmed from the 5-star XYZ stage record; shows stacked X, Y, and Z slides with helper-built box geometry.'
tags:
  - cadquery
  - examples
  - articulation
  - prismatic
  - xyz
  - cartesian
---
# XYZ Cartesian Stage

The 5-star XYZ stage record is a good reference when you want a compact but still believable three-axis stack without a heavy mesh-export layer.

```python
from sdk import ArticulatedObject, ArticulationType, Box, Inertial, MotionLimits, Origin


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="xyz_cartesian_stage")

    model.material("anodized_black", rgba=(0.16, 0.17, 0.20, 1.0))
    model.material("machined_aluminum", rgba=(0.72, 0.74, 0.77, 1.0))
    model.material("rail_steel", rgba=(0.56, 0.58, 0.62, 1.0))
    model.material("plate_blue", rgba=(0.22, 0.35, 0.66, 1.0))
    model.material("dark_trim", rgba=(0.10, 0.11, 0.13, 1.0))

    base = model.part("base")
    _add_box(base, (0.36, 0.25, 0.03), (0.0, 0.0, 0.015), "anodized_black")
    _add_box(base, (0.28, 0.024, 0.014), (0.0, -0.078, 0.037), "rail_steel")
    _add_box(base, (0.28, 0.024, 0.014), (0.0, 0.078, 0.037), "rail_steel")

    x_gantry = model.part("x_gantry")
    _add_box(x_gantry, (0.19, 0.20, 0.028), (0.0, 0.0, 0.021), "machined_aluminum")
    _add_box(x_gantry, (0.020, 0.180, 0.012), (-0.032, 0.0, 0.053), "rail_steel")
    _add_box(x_gantry, (0.020, 0.180, 0.012), (0.032, 0.0, 0.053), "rail_steel")

    y_saddle = model.part("y_saddle")
    _add_box(y_saddle, (0.112, 0.120, 0.024), (-0.016, 0.0, 0.018), "machined_aluminum")
    _add_box(y_saddle, (0.028, 0.024, 0.180), (0.021, -0.033, 0.120), "machined_aluminum")
    _add_box(y_saddle, (0.028, 0.024, 0.180), (0.021, 0.033, 0.120), "machined_aluminum")

    z_carriage = model.part("z_carriage")
    _add_box(z_carriage, (0.032, 0.076, 0.090), (0.019, 0.0, 0.0), "machined_aluminum")
    _add_box(z_carriage, (0.008, 0.078, 0.078), (0.075, 0.0, -0.038), "plate_blue")

    model.articulation("base_to_x", ArticulationType.PRISMATIC, parent=base, child=x_gantry, origin=Origin(xyz=(0.0, 0.0, 0.037)), axis=(1.0, 0.0, 0.0), motion_limits=MotionLimits(lower=-BASE_X_TRAVEL, upper=BASE_X_TRAVEL, effort=120.0, velocity=0.25))
    model.articulation("x_to_y", ArticulationType.PRISMATIC, parent=x_gantry, child=y_saddle, origin=Origin(xyz=(0.0, 0.0, 0.053)), axis=(0.0, 1.0, 0.0), motion_limits=MotionLimits(lower=-Y_STAGE_TRAVEL, upper=Y_STAGE_TRAVEL, effort=90.0, velocity=0.20))
    model.articulation("y_to_z", ArticulationType.PRISMATIC, parent=y_saddle, child=z_carriage, origin=Origin(xyz=(0.038, 0.0, 0.100)), axis=(0.0, 0.0, 1.0), motion_limits=MotionLimits(lower=-Z_STAGE_TRAVEL, upper=Z_STAGE_TRAVEL, effort=70.0, velocity=0.18))

    return model
```
