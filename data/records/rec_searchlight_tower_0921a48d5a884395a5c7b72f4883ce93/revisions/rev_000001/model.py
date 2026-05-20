from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


PAN_Z = 1.45
TILT_Z = 0.50


def _head_shell_mesh():
    """One low-cost molded searchlight housing: tube, front bezel, rear cap."""
    body = (
        cq.Workplane("YZ")
        .circle(0.180)
        .circle(0.145)
        .extrude(0.460)
        .translate((-0.200, 0.0, 0.0))
    )
    front_bezel = (
        cq.Workplane("YZ")
        .circle(0.195)
        .circle(0.135)
        .extrude(0.040)
        .translate((0.245, 0.0, 0.0))
    )
    rear_cap = (
        cq.Workplane("YZ")
        .circle(0.160)
        .extrude(0.035)
        .translate((-0.225, 0.0, 0.0))
    )
    service_flat = cq.Workplane("XY").box(0.170, 0.120, 0.020).translate((-0.115, 0.0, 0.176))
    return body.union(front_bezel).union(rear_cap).union(service_flat)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="cost_optimized_searchlight_tower")

    galvanized = Material("galvanized_steel", rgba=(0.55, 0.58, 0.56, 1.0))
    stamped = Material("stamped_base_dark", rgba=(0.12, 0.13, 0.13, 1.0))
    molded_black = Material("molded_black", rgba=(0.015, 0.017, 0.018, 1.0))
    bearing_gray = Material("bearing_zinc", rgba=(0.46, 0.47, 0.45, 1.0))
    bolt_zinc = Material("zinc_bolts", rgba=(0.72, 0.72, 0.68, 1.0))
    glass = Material("blue_tinted_lens", rgba=(0.25, 0.55, 0.95, 0.42))
    reflector = Material("bright_reflector", rgba=(0.92, 0.88, 0.72, 1.0))

    tower = model.part("tower")
    tower.visual(
        Box((0.58, 0.58, 0.040)),
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        material=stamped,
        name="base_plate",
    )
    # Four overlapping walls make the mast read as an inexpensive square tube extrusion.
    tower.visual(
        Box((0.120, 0.018, 1.340)),
        origin=Origin(xyz=(0.0, 0.051, 0.710)),
        material=galvanized,
        name="mast_wall_front",
    )
    tower.visual(
        Box((0.120, 0.018, 1.340)),
        origin=Origin(xyz=(0.0, -0.051, 0.710)),
        material=galvanized,
        name="mast_wall_rear",
    )
    tower.visual(
        Box((0.018, 0.120, 1.340)),
        origin=Origin(xyz=(0.051, 0.0, 0.710)),
        material=galvanized,
        name="mast_wall_side_0",
    )
    tower.visual(
        Box((0.018, 0.120, 1.340)),
        origin=Origin(xyz=(-0.051, 0.0, 0.710)),
        material=galvanized,
        name="mast_wall_side_1",
    )
    tower.visual(
        Box((0.210, 0.210, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, 1.3975)),
        material=stamped,
        name="top_cap",
    )
    tower.visual(
        Cylinder(radius=0.145, length=0.035),
        origin=Origin(xyz=(0.0, 0.0, 1.4325)),
        material=bearing_gray,
        name="top_bearing",
    )
    tower.visual(
        Box((0.185, 0.035, 0.090)),
        origin=Origin(xyz=(0.0, 0.068, 0.205)),
        material=stamped,
        name="snap_clamp_front",
    )
    tower.visual(
        Box((0.185, 0.035, 0.090)),
        origin=Origin(xyz=(0.0, -0.068, 0.205)),
        material=stamped,
        name="snap_clamp_rear",
    )
    for ix, x in enumerate((-0.215, 0.215)):
        for iy, y in enumerate((-0.215, 0.215)):
            tower.visual(
                Cylinder(radius=0.024, length=0.014),
                origin=Origin(xyz=(x, y, 0.047)),
                material=bolt_zinc,
                name=f"base_bolt_{ix}_{iy}",
            )

    pan_yoke = model.part("pan_yoke")
    pan_yoke.visual(
        Cylinder(radius=0.165, length=0.040),
        origin=Origin(xyz=(0.0, 0.0, 0.020)),
        material=molded_black,
        name="pan_disk",
    )
    pan_yoke.visual(
        Cylinder(radius=0.070, length=0.190),
        origin=Origin(xyz=(0.0, 0.0, 0.125)),
        material=molded_black,
        name="pan_spigot",
    )
    pan_yoke.visual(
        Box((0.160, 0.130, 0.250)),
        origin=Origin(xyz=(0.0, 0.0, 0.165)),
        material=molded_black,
        name="upright_web",
    )
    pan_yoke.visual(
        Box((0.170, 0.560, 0.060)),
        origin=Origin(xyz=(0.0, 0.0, 0.230)),
        material=molded_black,
        name="yoke_crossbar",
    )
    for suffix, y in (("pos", 0.250), ("neg", -0.250)):
        pan_yoke.visual(
            Box((0.085, 0.060, 0.290)),
            origin=Origin(xyz=(0.0, y, 0.380)),
            material=molded_black,
            name=f"yoke_arm_{suffix}",
        )
        pan_yoke.visual(
            Box((0.130, 0.080, 0.125)),
            origin=Origin(xyz=(0.0, y, TILT_Z)),
            material=molded_black,
            name=f"bearing_block_{suffix}",
        )
        pan_yoke.visual(
            Cylinder(radius=0.068, length=0.045),
            origin=Origin(xyz=(0.0, 0.311 if y > 0 else -0.311, TILT_Z), rpy=(-math.pi / 2, 0.0, 0.0)),
            material=bearing_gray,
            name=f"bearing_cap_{suffix}",
        )
    pan_yoke.visual(
        Box((0.060, 0.110, 0.030)),
        origin=Origin(xyz=(-0.112, 0.0, 0.045)),
        material=bolt_zinc,
        name="pan_index_tab",
    )

    head = model.part("head")
    head.visual(
        mesh_from_cadquery(_head_shell_mesh(), "head_shell"),
        material=molded_black,
        name="head_shell",
    )
    head.visual(
        Cylinder(radius=0.040, length=0.420),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-math.pi / 2, 0.0, 0.0)),
        material=bearing_gray,
        name="trunnion_axle",
    )
    head.visual(
        Cylinder(radius=0.138, length=0.012),
        origin=Origin(xyz=(0.267, 0.0, 0.0), rpy=(0.0, math.pi / 2, 0.0)),
        material=glass,
        name="front_lens",
    )
    head.visual(
        Cylinder(radius=0.147, length=0.010),
        origin=Origin(xyz=(0.247, 0.0, 0.0), rpy=(0.0, math.pi / 2, 0.0)),
        material=reflector,
        name="reflector_bowl",
    )
    head.visual(
        Box((0.150, 0.045, 0.028)),
        origin=Origin(xyz=(-0.080, 0.0, 0.200)),
        material=bolt_zinc,
        name="service_latch",
    )

    model.articulation(
        "pan",
        ArticulationType.REVOLUTE,
        parent=tower,
        child=pan_yoke,
        origin=Origin(xyz=(0.0, 0.0, PAN_Z)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=35.0, velocity=1.0, lower=-math.pi, upper=math.pi),
    )
    model.articulation(
        "tilt",
        ArticulationType.REVOLUTE,
        parent=pan_yoke,
        child=head,
        origin=Origin(xyz=(0.0, 0.0, TILT_Z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=0.8, lower=-0.70, upper=0.90),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    tower = object_model.get_part("tower")
    pan_yoke = object_model.get_part("pan_yoke")
    head = object_model.get_part("head")
    pan = object_model.get_articulation("pan")
    tilt = object_model.get_articulation("tilt")

    ctx.expect_gap(
        pan_yoke,
        tower,
        axis="z",
        positive_elem="pan_disk",
        negative_elem="top_bearing",
        max_gap=0.001,
        max_penetration=0.00001,
        name="pan disk sits on top bearing without sinking",
    )
    ctx.expect_overlap(
        pan_yoke,
        tower,
        axes="xy",
        elem_a="pan_disk",
        elem_b="top_bearing",
        min_overlap=0.10,
        name="pan bearing has wide coaxial support",
    )
    ctx.expect_gap(
        pan_yoke,
        head,
        axis="y",
        positive_elem="bearing_block_pos",
        negative_elem="trunnion_axle",
        max_gap=0.002,
        max_penetration=0.001,
        name="positive yoke bearing captures trunnion end",
    )
    ctx.expect_gap(
        head,
        pan_yoke,
        axis="y",
        positive_elem="trunnion_axle",
        negative_elem="bearing_block_neg",
        max_gap=0.002,
        max_penetration=0.001,
        name="negative yoke bearing captures trunnion end",
    )
    ctx.expect_gap(
        head,
        pan_yoke,
        axis="z",
        positive_elem="head_shell",
        negative_elem="yoke_crossbar",
        min_gap=0.010,
        name="head clears the stamped yoke crossbar",
    )

    lens_rest = ctx.part_element_world_aabb(head, elem="front_lens")
    rest_lens_z = None if lens_rest is None else (lens_rest[0][2] + lens_rest[1][2]) * 0.5
    rest_lens_y = None if lens_rest is None else (lens_rest[0][1] + lens_rest[1][1]) * 0.5
    with ctx.pose({tilt: 0.60}):
        lens_up = ctx.part_element_world_aabb(head, elem="front_lens")
    up_lens_z = None if lens_up is None else (lens_up[0][2] + lens_up[1][2]) * 0.5
    ctx.check(
        "positive tilt raises beam",
        rest_lens_z is not None and up_lens_z is not None and up_lens_z > rest_lens_z + 0.10,
        details=f"rest_z={rest_lens_z}, tilted_z={up_lens_z}",
    )

    with ctx.pose({pan: 0.70}):
        lens_panned = ctx.part_element_world_aabb(head, elem="front_lens")
    panned_lens_y = None if lens_panned is None else (lens_panned[0][1] + lens_panned[1][1]) * 0.5
    ctx.check(
        "pan rotates the searchlight around mast",
        rest_lens_y is not None and panned_lens_y is not None and panned_lens_y > rest_lens_y + 0.12,
        details=f"rest_y={rest_lens_y}, panned_y={panned_lens_y}",
    )

    return ctx.report()


object_model = build_object_model()
