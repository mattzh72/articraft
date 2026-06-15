---
title: 'Orchestra-style Music Stand with Tripod Base and Telescoping Pole'
description: 'Base SDK music stand example with a tripod base, telescoping center pole, broad sheet-music desk, retaining lip, and retained-insertion prismatic mast travel.'
tags:
  - sdk
  - base sdk
  - music stand
  - orchestra stand
  - sheet music stand
  - tripod base
  - telescoping pole
  - center pole
  - retaining lip
  - sheet music desk
  - prismatic articulation
  - fixed articulation
  - mesh from geometry
  - tube from spline points
  - tripod
  - stand
  - telescoping
  - retained insertion
---
# Orchestra-style Music Stand with Tripod Base and Telescoping Pole

This base-SDK example is a strong reference for a realistic orchestra-style music stand with a tripod base, telescoping center pole, broad sheet-music desk, and forward retaining lip. It is useful for queries such as `music stand`, `orchestra stand`, `sheet music stand`, `tripod base`, `telescoping pole`, `retaining lip`, `tube_from_spline_points`, `mesh_from_geometry`, and `prismatic articulation`.

The modeling patterns worth copying are:

- a retained-insertion prismatic mast whose authored length exceeds the visibly exposed pole so the upper travel limit still leaves real overlap.
- tripod legs built from `tube_from_spline_points(...)` instead of straight cylinders so the footprint reads as a real folded stand.
- a broad desk assembled from simple plates, flanges, and a forward retaining shelf rather than one oversized box.
- a small clamp/collar stack that makes the telescoping sleeve read as mechanically legible.
- mixed articulation types: prismatic pole extension with a fixed desk mount on the moving mast.

```python
from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Inertial,
    MotionLimits,
    Origin,
    Sphere,
    mesh_from_geometry,
    tube_from_spline_points,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="orchestra_music_stand")

    powder_black = model.material("powder_black", rgba=(0.16, 0.16, 0.17, 1.0))
    graphite = model.material("graphite", rgba=(0.23, 0.24, 0.26, 1.0))
    rubber = model.material("rubber", rgba=(0.08, 0.08, 0.09, 1.0))

    base = model.part("base")
    base.visual(
        Cylinder(radius=0.055, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, 0.100)),
        material=powder_black,
        name="tripod_hub",
    )
    base.visual(
        Cylinder(radius=0.028, length=0.060),
        origin=Origin(xyz=(0.0, 0.0, 0.155)),
        material=powder_black,
        name="lower_collar",
    )
    base.visual(
        Cylinder(radius=0.0145, length=0.340),
        origin=Origin(xyz=(0.0, 0.0, 0.350)),
        material=powder_black,
        name="outer_tube",
    )
    base.visual(
        Cylinder(radius=0.020, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, 0.545)),
        material=powder_black,
        name="top_socket",
    )
    base.visual(
        Cylinder(radius=0.006, length=0.028),
        origin=Origin(
            xyz=(0.030, 0.0, 0.550),
            rpy=(0.0, math.pi / 2.0, 0.0),
        ),
        material=graphite,
        name="clamp_stem",
    )
    base.visual(
        Sphere(radius=0.012),
        origin=Origin(xyz=(0.048, 0.0, 0.550)),
        material=rubber,
        name="clamp_knob",
    )

    leg_angles = (0.0, 2.0 * math.pi / 3.0, 4.0 * math.pi / 3.0)
    for index, angle in enumerate(leg_angles):
        c = math.cos(angle)
        s = math.sin(angle)
        leg_mesh = tube_from_spline_points(
            [
                (0.024 * c, 0.024 * s, 0.112),
                (0.170 * c, 0.170 * s, 0.070),
                (0.360 * c, 0.360 * s, 0.015),
            ],
            radius=0.010,
            samples_per_segment=18,
            radial_segments=18,
            cap_ends=True,
        )
        base.visual(
            mesh_from_geometry(leg_mesh, f"music_stand_leg_{index}"),
            material=powder_black,
            name=f"leg_{index}",
        )
        base.visual(
            Sphere(radius=0.012),
            origin=Origin(xyz=(0.360 * c, 0.360 * s, 0.015)),
            material=rubber,
            name=f"foot_{index}",
        )

    base.inertial = Inertial.from_geometry(
        Box((0.76, 0.76, 0.72)),
        mass=3.2,
        origin=Origin(xyz=(0.0, 0.0, 0.360)),
    )

    upper_pole = model.part("upper_pole")
    upper_pole.visual(
        # The inner mast extends below the visible seat so it stays engaged at max travel.
        Cylinder(radius=0.0105, length=1.020),
        origin=Origin(xyz=(0.0, 0.0, 0.140)),
        material=graphite,
        name="inner_tube",
    )
    upper_pole.visual(
        Cylinder(radius=0.015, length=0.030),
        origin=Origin(xyz=(0.0, 0.0, 0.665)),
        material=powder_black,
        name="top_cap",
    )
    upper_pole.inertial = Inertial.from_geometry(
        Box((0.040, 0.040, 1.050)),
        mass=1.0,
        origin=Origin(xyz=(0.0, 0.0, 0.155)),
    )

    model.articulation(
        "base_to_upper_pole",
        ArticulationType.PRISMATIC,
        parent=base,
        child=upper_pole,
        origin=Origin(xyz=(0.0, 0.0, 0.540)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=60.0,
            velocity=0.15,
            lower=0.0,
            upper=0.300,
        ),
    )

    desk = model.part("desk")
    desk.visual(
        Box((0.064, 0.030, 0.036)),
        origin=Origin(xyz=(0.0, 0.0, 0.018)),
        material=powder_black,
        name="receiver_block",
    )
    desk.visual(
        mesh_from_geometry(
            tube_from_spline_points(
                [
                    (0.0, 0.0, 0.008),
                    (0.0, -0.025, 0.055),
                    (0.0, -0.075, 0.135),
                    (0.0, -0.125, 0.205),
                ],
                radius=0.010,
                samples_per_segment=18,
                radial_segments=18,
                cap_ends=True,
            ),
            "music_stand_support_arm",
        ),
        material=powder_black,
        name="support_arm",
    )
    desk_angle = 0.33
    desk.visual(
        Box((0.540, 0.006, 0.350)),
        origin=Origin(xyz=(0.0, -0.145, 0.230), rpy=(desk_angle, 0.0, 0.0)),
        material=graphite,
        name="panel",
    )
    desk.visual(
        Box((0.100, 0.018, 0.220)),
        origin=Origin(xyz=(0.0, -0.141, 0.205), rpy=(desk_angle, 0.0, 0.0)),
        material=powder_black,
        name="center_reinforcement",
    )
    desk.visual(
        Box((0.014, 0.028, 0.350)),
        origin=Origin(xyz=(-0.263, -0.140, 0.230), rpy=(desk_angle, 0.0, 0.0)),
        material=graphite,
        name="left_flange",
    )
    desk.visual(
        Box((0.014, 0.028, 0.350)),
        origin=Origin(xyz=(0.263, -0.140, 0.230), rpy=(desk_angle, 0.0, 0.0)),
        material=graphite,
        name="right_flange",
    )
    desk.visual(
        Box((0.500, 0.022, 0.030)),
        origin=Origin(xyz=(0.0, -0.096, 0.076), rpy=(desk_angle, 0.0, 0.0)),
        material=graphite,
        name="lower_back_rail",
    )
    desk.visual(
        Box((0.500, 0.055, 0.012)),
        origin=Origin(xyz=(0.0, -0.012, 0.062), rpy=(0.08, 0.0, 0.0)),
        material=graphite,
        name="retaining_shelf",
    )
    desk.visual(
        Box((0.500, 0.006, 0.022)),
        origin=Origin(xyz=(0.0, 0.014, 0.068), rpy=(0.08, 0.0, 0.0)),
        material=graphite,
        name="retaining_fence",
    )
    desk.inertial = Inertial.from_geometry(
        Box((0.560, 0.220, 0.430)),
        mass=1.4,
        origin=Origin(xyz=(0.0, -0.090, 0.205)),
    )

    model.articulation(
        "upper_pole_to_desk",
        ArticulationType.FIXED,
        parent=upper_pole,
        child=desk,
        origin=Origin(xyz=(0.0, 0.0, 0.680)),
    )

    return model
```
