---
title: 'Artisan Stand Mixer with Tilt Head, Balloon Whisk, and Articulated Controls'
description: 'Base SDK stand mixer example with a lofted neck and head shell, lathed bowl, centered balloon whisk, articulated tilt head, rotating hub, sliding speed control, and pivoting tilt-lock lever.'
tags:
  - sdk
  - base sdk
  - stand mixer
  - artisan mixer
  - kitchen appliance
  - tilt head
  - mixing bowl
  - balloon whisk
  - whisk attachment
  - speed control
  - tilt lock
  - articulated controls
  - place on surface
  - section loft
  - lathe geometry
  - tube from spline points
  - revolute articulation
  - prismatic articulation
  - continuous articulation
---
# Artisan Stand Mixer with Tilt Head, Balloon Whisk, and Articulated Controls

This base-SDK example is a strong reference for a stylized but mechanically legible stand mixer with a sculpted body, forward-mounted bowl, realistic balloon whisk assembly, tilt-head motion, and articulated controls mounted directly to curved shell surfaces. It is useful for queries such as `stand mixer`, `artisan mixer`, `tilt head`, `balloon whisk`, `speed slider`, `tilt lock`, `place_on_surface`, `section_loft`, and `LatheGeometry`.

The modeling patterns worth copying are:

- `section_loft(...)` for a tapered neck and head housing instead of stacking boxes.
- `LatheGeometry.from_shell_profiles(...)` for a thin-walled mixing bowl.
- `tube_from_spline_points(...)` for repeated whisk loops with a separate ferrule and drive shaft.
- `place_on_surface(...)` for mounting controls onto the actual shell geometry rather than an approximate AABB face.
- mixed articulation types in one appliance: revolute head tilt and lock lever, continuous hub spin, and prismatic speed control travel.

```python
from __future__ import annotations

# User code should import every SDK/stdlib symbol it uses instead of relying on
# hidden scaffold imports.
# >>> USER_CODE_START
import math
from sdk import (
    ArticulatedObject,
    ArticulationType,
    AssetContext,
    Box,
    Cylinder,
    CylinderGeometry,
    ExtrudeGeometry,
    Inertial,
    LatheGeometry,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
    place_on_surface,
    rounded_rect_profile,
    section_loft,
    tube_from_spline_points,
)

ASSETS = AssetContext.from_script(__file__)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="artisan_mixer", assets=ASSETS)

    body_metal = model.material("body_metal", rgba=(0.7, 0.1, 0.1, 1.0))
    bowl_metal = model.material("bowl_metal", rgba=(0.9, 0.9, 0.9, 1.0))
    plastic_black = model.material("plastic_black", rgba=(0.1, 0.1, 0.1, 1.0))

    base_unit = model.part("base_unit")

    bp_profile = rounded_rect_profile(0.35, 0.22, 0.05)
    bp_geom = ExtrudeGeometry(bp_profile, 0.04)
    bp_mesh = mesh_from_geometry(bp_geom, ASSETS.mesh_path("base_plate.obj"))
    base_unit.visual(
        bp_mesh,
        origin=Origin(xyz=(0.075, 0.0, 0.02)),
        material=body_metal,
        name="base_plate",
    )

    neck_s0 = [(x, y, 0.0) for x, y in rounded_rect_profile(0.1, 0.12, 0.03)]
    neck_s1 = [(x, y, 0.18) for x, y in rounded_rect_profile(0.08, 0.1, 0.03)]
    neck_s2 = [(x, y, 0.35) for x, y in rounded_rect_profile(0.06, 0.08, 0.025)]
    neck_geom = section_loft([neck_s0, neck_s1, neck_s2])
    neck_mesh = mesh_from_geometry(neck_geom, ASSETS.mesh_path("neck.obj"))
    base_unit.visual(
        neck_mesh,
        origin=Origin(xyz=(-0.06, 0.0, 0.04)),
        material=body_metal,
        name="neck",
    )
    base_unit.inertial = Inertial.from_geometry(Box((0.35, 0.22, 0.39)), mass=6.0)

    bowl = model.part("bowl")
    outer_prof = [(0.02, 0.0), (0.06, 0.01), (0.1, 0.06), (0.1, 0.15), (0.105, 0.16)]
    inner_prof = [(0.0, 0.005), (0.055, 0.015), (0.095, 0.06), (0.095, 0.155)]
    bowl_geom = LatheGeometry.from_shell_profiles(outer_prof, inner_prof, segments=48)
    bowl_mesh = mesh_from_geometry(bowl_geom, ASSETS.mesh_path("bowl.obj"))
    bowl.visual(bowl_mesh, material=bowl_metal, name="bowl_shell")
    bowl.inertial = Inertial.from_geometry(Cylinder(radius=0.1, length=0.16), mass=0.8)

    model.articulation(
        "base_to_bowl",
        ArticulationType.FIXED,
        parent=base_unit,
        child=bowl,
        origin=Origin(xyz=(0.12, 0.0, 0.04)),
    )

    head = model.part("head")

    def yz_section(w: float, h: float, r: float, x: float) -> list[tuple[float, float, float]]:
        return [(x, y, z) for z, y in rounded_rect_profile(h, w, r)]

    head_s0 = yz_section(0.08, 0.1, 0.03, 0.0)
    head_s1 = yz_section(0.14, 0.16, 0.05, 0.15)
    head_s2 = yz_section(0.1, 0.12, 0.04, 0.35)
    head_geom = section_loft([head_s0, head_s1, head_s2])
    head_mesh = mesh_from_geometry(head_geom, ASSETS.mesh_path("head.obj"))
    head.visual(head_mesh, material=body_metal, name="head_shell")
    head.inertial = Inertial.from_geometry(
        Box((0.35, 0.16, 0.16)),
        mass=4.0,
        origin=Origin(xyz=(0.175, 0.0, 0.0)),
    )

    tilt_lock = model.part("tilt_lock")
    tilt_lock.visual(
        Cylinder(radius=0.006, length=0.008),
        origin=Origin(xyz=(0.0, 0.0, 0.004)),
        material=plastic_black,
        name="tilt_lock_pivot",
    )
    tilt_lock.visual(
        Cylinder(radius=0.0035, length=0.020),
        origin=Origin(xyz=(0.012, 0.0, 0.010), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=plastic_black,
        name="tilt_lock_handle",
    )
    tilt_lock.visual(
        Cylinder(radius=0.0038, length=0.012),
        origin=Origin(xyz=(0.020, 0.0, 0.016)),
        material=plastic_black,
        name="tilt_lock_tip",
    )
    tilt_lock.inertial = Inertial.from_geometry(
        Box((0.030, 0.010, 0.028)),
        mass=0.05,
        origin=Origin(xyz=(0.014, 0.0, 0.012)),
    )

    speed_control = model.part("speed_control")
    speed_control.visual(
        Box((0.014, 0.008, 0.006)),
        origin=Origin(xyz=(0.0, 0.0, 0.003)),
        material=plastic_black,
        name="speed_slider_base",
    )
    speed_control.visual(
        Cylinder(radius=0.0035, length=0.016),
        origin=Origin(xyz=(0.0, 0.0, 0.014)),
        material=plastic_black,
        name="speed_slider_stem",
    )
    speed_control.visual(
        Cylinder(radius=0.004, length=0.016),
        origin=Origin(xyz=(0.006, 0.0, 0.021), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=plastic_black,
        name="speed_slider_grip",
    )
    speed_control.inertial = Inertial.from_geometry(
        Box((0.026, 0.010, 0.030)),
        mass=0.05,
        origin=Origin(xyz=(0.006, 0.0, 0.015)),
    )

    model.articulation(
        "neck_to_head",
        ArticulationType.REVOLUTE,
        parent=base_unit,
        child=head,
        origin=Origin(xyz=(-0.06, 0.0, 0.39)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=15.0,
            velocity=1.0,
            lower=0.0,
            upper=math.radians(65),
        ),
    )

    model.articulation(
        "base_to_tilt_lock",
        ArticulationType.REVOLUTE,
        parent=base_unit,
        child=tilt_lock,
        origin=place_on_surface(
            tilt_lock,
            base_unit,
            point_hint=(-0.055, 0.055, 0.255),
            asset_root=ASSETS,
            prefer_collisions=False,
            child_prefer_collisions=False,
        ),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.5, velocity=2.0, lower=-0.35, upper=0.35),
    )

    model.articulation(
        "head_to_speed_control",
        ArticulationType.PRISMATIC,
        parent=head,
        child=speed_control,
        origin=place_on_surface(
            speed_control,
            head,
            point_hint=(0.145, 0.075, 0.005),
            asset_root=ASSETS,
            prefer_collisions=False,
            child_prefer_collisions=False,
        ),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=1.0, velocity=0.08, lower=-0.012, upper=0.012),
    )

    hub = model.part("hub")
    hub.visual(
        Cylinder(radius=0.04, length=0.05),
        origin=Origin(xyz=(0.0, 0.0, -0.025)),
        material=plastic_black,
        name="hub_shell",
    )
    hub.inertial = Inertial.from_geometry(Cylinder(radius=0.04, length=0.05), mass=0.3)

    model.articulation(
        "head_to_hub",
        ArticulationType.CONTINUOUS,
        parent=head,
        child=hub,
        origin=Origin(xyz=(0.18, 0.0, -0.05)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=10.0, velocity=15.0),
    )

    whisk = model.part("whisk")
    whisk_wires = []
    num_loops = 8
    for i in range(num_loops):
        angle = i * math.pi / num_loops
        c, s = math.cos(angle), math.sin(angle)
        pts = [
            (0.009 * c, 0.009 * s, -0.050),
            (0.020 * c, 0.020 * s, -0.068),
            (0.034 * c, 0.034 * s, -0.100),
            (0.045 * c, 0.045 * s, -0.132),
            (0.0, 0.0, -0.155),
            (-0.045 * c, -0.045 * s, -0.132),
            (-0.034 * c, -0.034 * s, -0.100),
            (-0.020 * c, -0.020 * s, -0.068),
            (-0.009 * c, -0.009 * s, -0.050),
        ]
        whisk_wires.append(tube_from_spline_points(pts, radius=0.0016))

    final_whisk_geom = CylinderGeometry(radius=0.0055, height=0.038).translate(0.0, 0.0, -0.019)
    final_whisk_geom.merge(CylinderGeometry(radius=0.0085, height=0.022).translate(0.0, 0.0, -0.045))
    final_whisk_geom.merge(CylinderGeometry(radius=0.012, height=0.020).translate(0.0, 0.0, -0.066))
    for w_geom in whisk_wires[1:]:
        final_whisk_geom.merge(w_geom)
    final_whisk_geom.merge(whisk_wires[0])

    whisk_mesh = mesh_from_geometry(final_whisk_geom, ASSETS.mesh_path("whisk.obj"))
    whisk.visual(whisk_mesh, material=bowl_metal, name="whisk_shell")
    whisk.inertial = Inertial.from_geometry(Cylinder(radius=0.05, length=0.16), mass=0.2)

    model.articulation(
        "hub_to_whisk",
        ArticulationType.FIXED,
        parent=hub,
        child=whisk,
        origin=Origin(xyz=(0.0, 0.0, -0.05)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model, asset_root=ASSETS.asset_root)
    ctx.check_model_valid()
    ctx.check_mesh_files_exist()

    base_unit = object_model.get_part("base_unit")
    bowl = object_model.get_part("bowl")
    head = object_model.get_part("head")
    hub = object_model.get_part("hub")
    whisk = object_model.get_part("whisk")
    tilt_lock = object_model.get_part("tilt_lock")
    speed_control = object_model.get_part("speed_control")

    neck_to_head = object_model.get_articulation("neck_to_head")
    base_to_tilt_lock = object_model.get_articulation("base_to_tilt_lock")
    head_to_speed_control = object_model.get_articulation("head_to_speed_control")

    ctx.fail_if_isolated_parts()
    ctx.warn_if_part_contains_disconnected_geometry_islands()

    ctx.allow_overlap(base_unit, head, reason="Hinge interpenetration")
    ctx.allow_overlap(bowl, whisk, reason="Whisk inside bowl")
    ctx.allow_overlap(head, hub, reason="Hub seated in head")
    ctx.allow_overlap(
        base_unit,
        tilt_lock,
        reason="Tilt lock pivot sits in a shallow neck-side recess.",
    )
    ctx.fail_if_parts_overlap_in_current_pose()

    ctx.expect_contact(bowl, base_unit)
    ctx.expect_contact(head, base_unit)
    ctx.expect_contact(hub, head)
    ctx.expect_contact(whisk, hub)
    ctx.expect_contact(tilt_lock, base_unit)
    ctx.expect_contact(speed_control, head)

    ctx.expect_within(bowl, base_unit, axes="xy")
    ctx.expect_overlap(whisk, bowl, axes="xy", min_overlap=0.01)

    speed_rest = ctx.part_world_position(speed_control)
    assert speed_rest is not None
    with ctx.pose({head_to_speed_control: 0.012}):
        speed_high = ctx.part_world_position(speed_control)
        assert speed_high is not None
        assert speed_high[2] > speed_rest[2] + 0.01
        ctx.expect_contact(speed_control, head)

    tilt_rest_aabb = ctx.part_world_aabb(tilt_lock)
    assert tilt_rest_aabb is not None
    with ctx.pose({base_to_tilt_lock: 0.30}):
        tilt_open_aabb = ctx.part_world_aabb(tilt_lock)
        assert tilt_open_aabb is not None
        assert tilt_open_aabb[0][0] < tilt_rest_aabb[0][0] - 0.003
        ctx.expect_contact(tilt_lock, base_unit)

    with ctx.pose({neck_to_head: math.radians(60)}):
        ctx.expect_gap(whisk, bowl, axis="z", min_gap=0.05)

    return ctx.report()


object_model = build_object_model()
# >>> USER_CODE_END
```
