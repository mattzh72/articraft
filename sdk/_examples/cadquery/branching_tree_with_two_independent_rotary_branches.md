---
title: 'Branching Tree with Two Independent Rotary Branches'
description: 'Trimmed from the 5-star Y-tree record; shows two independent branches driven from one trunk.'
tags:
  - cadquery
  - examples
  - articulation
  - branching
  - revolute
  - tree
---
# Branching Tree with Two Independent Rotary Branches

This excerpt keeps the real trunk-plus-two-branches split from the 5-star Y-tree record. It is the simplest clean example of a non-serial articulation graph.

```python
from sdk import ArticulatedObject, ArticulationType, Box, Inertial, MotionLimits, Origin, mesh_from_cadquery


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="mechanical_y_tree", assets=ASSETS)

    model.material("frame_dark", rgba=(0.22, 0.24, 0.27, 1.0))
    model.material("branch_green", rgba=(0.31, 0.54, 0.36, 1.0))

    trunk = model.part("trunk")
    trunk.visual(mesh_from_cadquery(_build_trunk_shape(), "trunk.obj", assets=ASSETS), material="frame_dark")

    left_branch = model.part("left_branch")
    left_branch.visual(mesh_from_cadquery(_build_branch_shape("left"), "left_branch.obj", assets=ASSETS), material="branch_green")

    right_branch = model.part("right_branch")
    right_branch.visual(mesh_from_cadquery(_build_branch_shape("right"), "right_branch.obj", assets=ASSETS), material="branch_green")

    model.articulation("trunk_to_left_branch", ArticulationType.REVOLUTE, parent=trunk, child=left_branch, origin=Origin(xyz=(-HUB_X, 0.0, HUB_Z)), axis=(0.0, 1.0, 0.0), motion_limits=MotionLimits(lower=-0.24, upper=0.24, effort=4.0, velocity=1.5))
    model.articulation("trunk_to_right_branch", ArticulationType.REVOLUTE, parent=trunk, child=right_branch, origin=Origin(xyz=(HUB_X, 0.0, HUB_Z)), axis=(0.0, 1.0, 0.0), motion_limits=MotionLimits(lower=-0.24, upper=0.24, effort=4.0, velocity=1.5))

    return model
```
