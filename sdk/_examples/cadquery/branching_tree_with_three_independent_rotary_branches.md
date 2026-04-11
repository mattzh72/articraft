---
title: 'Branching Tree with Three Independent Rotary Branches'
description: 'Trimmed from the 5-star three-branch rotary tree record; shows repeated rotary arms around one hub.'
tags:
  - cadquery
  - examples
  - articulation
  - branching
  - revolute
  - hub
---
# Branching Tree with Three Independent Rotary Branches

The three-branch 5-star record is useful because it shows how to scale a branching topology with a small loop instead of writing each articulation by hand.

```python
from math import cos, sin

from sdk import ArticulatedObject, ArticulationType, Cylinder, Inertial, Origin, mesh_from_cadquery


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="rotary_tree_mechanism", assets=ASSETS)

    model.material("hub_gray", rgba=(0.58, 0.60, 0.63, 1.0))
    model.material("arm_blue", rgba=(0.23, 0.45, 0.72, 1.0))

    hub_mesh = mesh_from_cadquery(_build_hub_shape(), "rotary_tree_hub.obj", assets=ASSETS)
    arm_mesh = mesh_from_cadquery(_build_arm_shape(), "rotary_tree_arm.obj", assets=ASSETS)

    hub = model.part("hub")
    hub.visual(hub_mesh, material="hub_gray")
    hub.inertial = Inertial.from_geometry(Cylinder(radius=HUB_RADIUS, length=HUB_HEIGHT), mass=1.15)

    for name, angle in zip(BRANCH_NAMES, BRANCH_ANGLES):
        branch = _add_branch_part(model, arm_mesh, name)
        model.articulation(
            f"hub_to_{name}",
            ArticulationType.REVOLUTE,
            parent=hub,
            child=branch,
            origin=Origin(xyz=(JOINT_RADIUS * cos(angle), JOINT_RADIUS * sin(angle), 0.0), rpy=(0.0, 0.0, angle)),
            axis=(0.0, -1.0, 0.0),
            motion_limits=ARM_LIMITS,
        )

    return model
```
