---
title: 'Dual Independent Finger Chains'
description: 'Trimmed from the 5-star dual-finger gripper record; shows two uncoupled phalanx chains mounted to one palm.'
tags:
  - cadquery
  - examples
  - articulation
  - revolute
  - finger
  - gripper
---
# Dual Independent Finger Chains

This excerpt keeps the exact left and right finger decomposition from the 5-star dual-finger record. It is a good pattern for branching but still symmetric kinematics.

```python
from sdk import ArticulatedObject, ArticulationType, MotionLimits, Origin


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="dual_finger_gripper", assets=ASSETS)

    model.material("palm_gray", rgba=(0.24, 0.26, 0.30, 1.0))
    model.material("finger_aluminum", rgba=(0.72, 0.75, 0.79, 1.0))

    palm = _add_mesh_link(model=model, name="palm", shape=_make_palm_shape(), mesh_name="palm.obj", collision_size=PALM_SIZE, collision_origin=Origin(), mass=0.45, material="palm_gray")
    left_proximal = _add_mesh_link(model=model, name="left_proximal", shape=_make_link_shape(PROXIMAL_SIZE), mesh_name="left_proximal.obj", collision_size=PROXIMAL_SIZE, collision_origin=proximal_origin, mass=0.08, material="finger_aluminum")
    left_distal = _add_mesh_link(model=model, name="left_distal", shape=_make_link_shape(DISTAL_SIZE), mesh_name="left_distal.obj", collision_size=DISTAL_SIZE, collision_origin=distal_origin, mass=0.05, material="finger_aluminum")
    right_proximal = _add_mesh_link(model=model, name="right_proximal", shape=_make_link_shape(PROXIMAL_SIZE), mesh_name="right_proximal.obj", collision_size=PROXIMAL_SIZE, collision_origin=proximal_origin, mass=0.08, material="finger_aluminum")
    right_distal = _add_mesh_link(model=model, name="right_distal", shape=_make_link_shape(DISTAL_SIZE), mesh_name="right_distal.obj", collision_size=DISTAL_SIZE, collision_origin=distal_origin, mass=0.05, material="finger_aluminum")

    model.articulation("palm_to_left_proximal", ArticulationType.REVOLUTE, parent=palm, child=left_proximal, origin=Origin(xyz=(-ROOT_X_OFFSET, ROOT_Y_OFFSET, root_z)), axis=(0.0, 0.0, -1.0), motion_limits=MotionLimits(lower=0.0, upper=0.42, effort=3.0, velocity=3.0))
    model.articulation("left_proximal_to_left_distal", ArticulationType.REVOLUTE, parent=left_proximal, child=left_distal, origin=Origin(xyz=(0.0, PROXIMAL_SIZE[1], 0.0)), axis=(0.0, 0.0, -1.0), motion_limits=MotionLimits(lower=0.0, upper=0.34, effort=2.0, velocity=3.0))
    model.articulation("palm_to_right_proximal", ArticulationType.REVOLUTE, parent=palm, child=right_proximal, origin=Origin(xyz=(ROOT_X_OFFSET, ROOT_Y_OFFSET, root_z)), axis=(0.0, 0.0, 1.0), motion_limits=MotionLimits(lower=0.0, upper=0.42, effort=3.0, velocity=3.0))
    model.articulation("right_proximal_to_right_distal", ArticulationType.REVOLUTE, parent=right_proximal, child=right_distal, origin=Origin(xyz=(0.0, PROXIMAL_SIZE[1], 0.0)), axis=(0.0, 0.0, 1.0), motion_limits=MotionLimits(lower=0.0, upper=0.34, effort=2.0, velocity=3.0))

    return model
```
