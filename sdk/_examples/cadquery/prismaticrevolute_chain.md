---
title: 'Prismatic-revolute Chain'
description: 'Trimmed from the 5-star inspection-fixture record; shows a sliding carriage that carries a hinged flap.'
tags:
  - cadquery
  - examples
  - articulation
  - prismatic
  - revolute
  - fixture
---
# Prismatic-revolute Chain

The 5-star inspection-fixture record is a good template for this topology because it separates the guided base motion from the distal flap motion cleanly.

```python
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Inertial,
    MotionLimits,
    Origin,
    mesh_from_cadquery,
)


def build_object_model() -> ArticulatedObject:
    cq = _require_cadquery()

    model = ArticulatedObject(name="inspection_fixture", assets=ASSETS)
    model.material("fixture_base", rgba=(0.42, 0.44, 0.48, 1.0))
    model.material("fixture_carriage", rgba=(0.78, 0.46, 0.14, 1.0))
    model.material("fixture_flap", rgba=(0.92, 0.77, 0.16, 1.0))

    base = model.part("base")
    base.visual(mesh_from_cadquery(_build_base_shape(cq), MESH_DIR / "fixture_base.obj"), material="fixture_base")
    base.inertial = Inertial.from_geometry(
        Box((BASE_L, BASE_W, 0.08)),
        mass=11.0,
        origin=Origin(xyz=(0.0, 0.0, 0.03)),
    )

    carriage = model.part("carriage")
    carriage.visual(mesh_from_cadquery(_build_carriage_shape(cq), MESH_DIR / "carriage.obj"), material="fixture_carriage")

    flap = model.part("flap")
    flap.visual(mesh_from_cadquery(_build_flap_shape(cq), MESH_DIR / "inspection_flap.obj"), material="fixture_flap")

    model.articulation(
        "base_to_carriage",
        ArticulationType.PRISMATIC,
        parent="base",
        child="carriage",
        origin=Origin(xyz=(0.0, RAIL_Y, (BASE_T / 2.0) + RAIL_H)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(
            lower=PRISMATIC_LOWER,
            upper=PRISMATIC_UPPER,
            effort=120.0,
            velocity=0.25,
        ),
    )
    model.articulation(
        "carriage_to_flap",
        ArticulationType.REVOLUTE,
        parent="carriage",
        child="flap",
        origin=Origin(xyz=(EAR_X, -RAIL_Y, EAR_Z)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(lower=0.0, upper=FLAP_UPPER, effort=10.0, velocity=1.5),
    )

    return model
```
