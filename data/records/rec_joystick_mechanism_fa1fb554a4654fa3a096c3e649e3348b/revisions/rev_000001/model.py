from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    CapsuleGeometry,
    Cylinder,
    LatheGeometry,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
    TorusGeometry,
    mesh_from_geometry,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="cardan_ring_joystick")

    cast_gray = model.material("cast_gray", rgba=(0.28, 0.30, 0.32, 1.0))
    dark_cast = model.material("dark_cast", rgba=(0.10, 0.11, 0.12, 1.0))
    brushed_steel = model.material("brushed_steel", rgba=(0.64, 0.66, 0.68, 1.0))
    bearing_black = model.material("bearing_black", rgba=(0.03, 0.035, 0.04, 1.0))
    rubber = model.material("rubber", rgba=(0.015, 0.015, 0.017, 1.0))

    cup_shell = mesh_from_geometry(
        LatheGeometry.from_shell_profiles(
            [
                (0.055, -0.245),
                (0.095, -0.225),
                (0.142, -0.150),
                (0.172, -0.060),
                (0.174, -0.036),
            ],
            [
                (0.022, -0.205),
                (0.062, -0.190),
                (0.105, -0.120),
                (0.138, -0.052),
                (0.148, -0.036),
            ],
            segments=72,
            start_cap="round",
            end_cap="flat",
            lip_samples=8,
        ),
        "cup_shell",
    )
    cup_rim = mesh_from_geometry(
        TorusGeometry(radius=0.160, tube=0.006, radial_segments=16, tubular_segments=72),
        "cup_rim",
    )
    base_bearing = mesh_from_geometry(
        TorusGeometry(radius=0.018, tube=0.008, radial_segments=14, tubular_segments=40),
        "base_bearing",
    )
    outer_hoop_mesh = mesh_from_geometry(
        TorusGeometry(radius=0.110, tube=0.013, radial_segments=18, tubular_segments=80),
        "outer_hoop",
    )
    outer_bearing_mesh = mesh_from_geometry(
        TorusGeometry(radius=0.021, tube=0.006, radial_segments=14, tubular_segments=40),
        "outer_inner_bearing",
    )
    inner_hoop_mesh = mesh_from_geometry(
        TorusGeometry(radius=0.062, tube=0.009, radial_segments=16, tubular_segments=64),
        "inner_hoop",
    )
    grip_cap_mesh = mesh_from_geometry(
        CapsuleGeometry(radius=0.024, length=0.060, radial_segments=24, height_segments=8),
        "rubber_grip",
    )

    base = model.part("cup_base")
    base.visual(cup_shell, material=cast_gray, name="cup_shell")
    base.visual(
        Cylinder(radius=0.205, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, -0.252)),
        material=dark_cast,
        name="floor_flange",
    )
    base.visual(cup_rim, origin=Origin(xyz=(0.0, 0.0, -0.037)), material=dark_cast, name="rolled_rim")
    for side, x in (("pos", 0.198), ("neg", -0.198)):
        base.visual(
            Box((0.040, 0.090, 0.174)),
            origin=Origin(xyz=(x, 0.0, -0.072)),
            material=cast_gray,
            name=f"yoke_post_{side}",
        )
        base.visual(
            Box((0.060, 0.056, 0.032)),
            origin=Origin(xyz=(0.5 * (x + math.copysign(0.150, x)), 0.0, -0.030)),
            material=cast_gray,
            name=f"rim_bridge_{side}",
        )
        base.visual(
            base_bearing,
            origin=Origin(xyz=(math.copysign(0.174, x), 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=bearing_black,
            name=f"outer_axis_bearing_{side}",
        )

    outer_ring = model.part("outer_ring")
    outer_ring.visual(outer_hoop_mesh, material=brushed_steel, name="outer_hoop")
    for side, x in (("pos", 0.138), ("neg", -0.138)):
        outer_ring.visual(
            Cylinder(radius=0.0105, length=0.072),
            origin=Origin(xyz=(x, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=bearing_black,
            name=f"outer_trunnion_{side}",
        )
    for side, y in (("pos", 0.096), ("neg", -0.096)):
        outer_ring.visual(
            outer_bearing_mesh,
            origin=Origin(xyz=(0.0, y, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=bearing_black,
            name=f"inner_axis_bearing_{side}",
        )

    inner_ring = model.part("inner_ring")
    inner_ring.visual(inner_hoop_mesh, material=brushed_steel, name="inner_hoop")
    inner_ring.visual(
        Cylinder(radius=0.026, length=0.024),
        origin=Origin(xyz=(0.0, 0.0, 0.012)),
        material=brushed_steel,
        name="central_collar",
    )
    inner_ring.visual(
        Box((0.126, 0.010, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=brushed_steel,
        name="spoke_x",
    )
    inner_ring.visual(
        Box((0.010, 0.126, 0.010)),
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        material=brushed_steel,
        name="spoke_y",
    )
    for side, y in (("pos", 0.088), ("neg", -0.088)):
        inner_ring.visual(
            Cylinder(radius=0.0075, length=0.075),
            origin=Origin(xyz=(0.0, y, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=bearing_black,
            name=f"inner_trunnion_{side}",
        )

    stick = model.part("stick")
    stick.visual(
        Cylinder(radius=0.012, length=0.300),
        origin=Origin(xyz=(0.0, 0.0, 0.174)),
        material=brushed_steel,
        name="shaft",
    )
    stick.visual(
        Cylinder(radius=0.017, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, 0.334)),
        material=rubber,
        name="grip_neck",
    )
    stick.visual(
        grip_cap_mesh,
        origin=Origin(xyz=(0.0, 0.0, 0.390)),
        material=rubber,
        name="hand_grip",
    )

    model.articulation(
        "outer_axis",
        ArticulationType.REVOLUTE,
        parent=base,
        child=outer_ring,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=3.0, lower=-0.55, upper=0.55),
    )
    model.articulation(
        "inner_axis",
        ArticulationType.REVOLUTE,
        parent=outer_ring,
        child=inner_ring,
        origin=Origin(),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=3.0, lower=-0.55, upper=0.55),
    )
    model.articulation(
        "stick_mount",
        ArticulationType.FIXED,
        parent=inner_ring,
        child=stick,
        origin=Origin(),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    outer = object_model.get_part("outer_ring")
    inner = object_model.get_part("inner_ring")
    stick = object_model.get_part("stick")
    outer_axis = object_model.get_articulation("outer_axis")
    inner_axis = object_model.get_articulation("inner_axis")

    base = object_model.get_part("cup_base")
    for side in ("pos", "neg"):
        ctx.allow_overlap(
            base,
            outer,
            elem_a=f"outer_axis_bearing_{side}",
            elem_b=f"outer_trunnion_{side}",
            reason="The outer trunnion shaft is intentionally captured by the cup-base bearing washer.",
        )
        ctx.expect_overlap(
            outer,
            base,
            axes="x",
            elem_a=f"outer_trunnion_{side}",
            elem_b=f"outer_axis_bearing_{side}",
            min_overlap=0.006,
            name=f"outer trunnion {side} remains inserted in the base bearing",
        )
        ctx.expect_within(
            outer,
            base,
            axes="yz",
            inner_elem=f"outer_trunnion_{side}",
            outer_elem=f"outer_axis_bearing_{side}",
            margin=0.001,
            name=f"outer trunnion {side} is centered in the base bearing",
        )

    for elem in ("inner_trunnion_pos", "inner_trunnion_neg"):
        ctx.allow_overlap(
            outer,
            inner,
            elem_a="outer_hoop",
            elem_b=elem,
            reason="The inner ring trunnion is intentionally captured through a simplified solid bearing boss on the outer ring.",
        )
        ctx.expect_overlap(
            inner,
            outer,
            axes="y",
            elem_a=elem,
            elem_b="outer_hoop",
            min_overlap=0.012,
            name=f"{elem} remains inserted through the outer ring bearing",
        )

    ctx.check(
        "cardan axes intersect at the gimbal center",
        outer_axis.origin.xyz == (0.0, 0.0, 0.0) and inner_axis.origin.xyz == (0.0, 0.0, 0.0),
        details=f"outer={outer_axis.origin.xyz}, inner={inner_axis.origin.xyz}",
    )
    ctx.check(
        "cardan axes are orthogonal",
        abs(sum(a * b for a, b in zip(outer_axis.axis, inner_axis.axis))) < 1e-6,
        details=f"outer_axis={outer_axis.axis}, inner_axis={inner_axis.axis}",
    )
    ctx.expect_gap(
        stick,
        inner,
        axis="z",
        positive_elem="shaft",
        negative_elem="central_collar",
        max_gap=0.001,
        max_penetration=0.000001,
        name="stick shaft seats on the inner ring collar",
    )

    neutral_aabb = ctx.part_element_world_aabb(stick, elem="shaft")
    with ctx.pose({outer_axis: 0.45}):
        tilted_aabb = ctx.part_element_world_aabb(stick, elem="shaft")
    ctx.check(
        "outer ring rotation tilts the stick about the x axis",
        neutral_aabb is not None
        and tilted_aabb is not None
        and ((tilted_aabb[0][1] + tilted_aabb[1][1]) * 0.5) < -0.045,
        details=f"neutral={neutral_aabb}, tilted={tilted_aabb}",
    )

    with ctx.pose({inner_axis: 0.45}):
        inner_tilt_aabb = ctx.part_element_world_aabb(stick, elem="shaft")
    ctx.check(
        "inner ring rotation tilts the stick about the y axis",
        neutral_aabb is not None
        and inner_tilt_aabb is not None
        and ((inner_tilt_aabb[0][0] + inner_tilt_aabb[1][0]) * 0.5) > 0.045,
        details=f"neutral={neutral_aabb}, tilted={inner_tilt_aabb}",
    )

    return ctx.report()


object_model = build_object_model()
