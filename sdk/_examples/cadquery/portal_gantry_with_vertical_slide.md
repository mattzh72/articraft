---
title: 'Portal Gantry with Vertical Slide'
description: 'Trimmed from the 5-star portal-gantry record; shows a beam carriage with a hanging Z slide.'
tags:
  - cadquery
  - examples
  - articulation
  - prismatic
  - gantry
  - portal
---
# Portal Gantry with Vertical Slide

This excerpt keeps the real three-part split from the 5-star gantry record: rigid portal frame, bridge carriage, and a separate hanging slide.

```python
from math import pi

from sdk import ArticulatedObject, ArticulationType, Box, Cylinder, Inertial, MotionLimits, Origin


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="portal_gantry_axis", assets=ASSETS)

    model.material("frame_aluminum", rgba=(0.74, 0.75, 0.77, 1.0))
    model.material("rail_steel", rgba=(0.28, 0.30, 0.33, 1.0))
    model.material("carriage_body", rgba=(0.18, 0.20, 0.23, 1.0))
    model.material("slide_body", rgba=(0.86, 0.87, 0.88, 1.0))
    model.material("accent_blue", rgba=(0.15, 0.42, 0.78, 1.0))
    model.material("tool_steel", rgba=(0.42, 0.44, 0.48, 1.0))

    frame = model.part("frame")
    _add_box(frame, (0.62, 0.12, 0.08), (0.0, 0.0, 0.66), "frame_aluminum")
    _add_box(frame, (0.56, 0.012, 0.018), (0.0, 0.054, 0.682), "rail_steel")
    _add_box(frame, (0.56, 0.012, 0.018), (0.0, 0.054, 0.638), "rail_steel")

    carriage = model.part("carriage")
    _add_box(carriage, (0.17, 0.086, 0.14), (0.0, 0.0, 0.0), "carriage_body")
    _add_box(carriage, (0.16, 0.052, 0.44), (0.0, 0.018, -0.29), "slide_body")
    _add_box(carriage, (0.024, 0.014, 0.40), (-0.05, 0.032, -0.27), "rail_steel")
    _add_box(carriage, (0.024, 0.014, 0.40), (0.05, 0.032, -0.27), "rail_steel")

    slide = model.part("slide")
    _add_box(slide, (0.13, 0.036, 0.26), (0.0, 0.0, -0.12), "slide_body")
    _add_box(slide, (0.082, 0.054, 0.11), (0.0, 0.018, -0.305), "carriage_body")
    _add_cylinder(slide, radius=0.018, length=0.045, xyz=(0.0, 0.045, -0.33), material="tool_steel", rpy=(pi / 2.0, 0.0, 0.0))

    model.articulation("frame_to_carriage", ArticulationType.PRISMATIC, parent=frame, child=carriage, origin=Origin(xyz=(0.0, 0.055, 0.66)), axis=(1.0, 0.0, 0.0), motion_limits=MotionLimits(lower=-0.13, upper=0.13, effort=450.0, velocity=0.8))
    model.articulation("carriage_to_slide", ArticulationType.PRISMATIC, parent=carriage, child=slide, origin=Origin(xyz=(0.0, 0.032, -0.08)), axis=(0.0, 0.0, -1.0), motion_limits=MotionLimits(lower=0.0, upper=0.18, effort=220.0, velocity=0.5))

    return model
```
