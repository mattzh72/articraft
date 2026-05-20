from __future__ import annotations

import math

import cadquery as cq
from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_cadquery,
)


def _square_tube(outer: float, inner: float, z_min: float, z_max: float) -> cq.Workplane:
    """Open-ended square tube authored in meters around the local Z axis."""
    length = z_max - z_min
    wall = (outer - inner) * 0.5
    if length <= 0.0 or wall <= 0.0:
        raise ValueError("tube must have positive length and wall thickness")

    outside = (
        cq.Workplane("XY")
        .box(outer, outer, length, centered=(True, True, False))
        .translate((0.0, 0.0, z_min))
    )
    cutter = (
        cq.Workplane("XY")
        .box(inner, inner, length + 0.02, centered=(True, True, False))
        .translate((0.0, 0.0, z_min - 0.01))
    )
    return outside.cut(cutter)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="telescoping_service_pole")

    black_paint = model.material("satin_black_powdercoat", rgba=(0.02, 0.022, 0.024, 1.0))
    dark_rubber = model.material("dark_rubber", rgba=(0.004, 0.004, 0.004, 1.0))
    brushed_steel = model.material("brushed_steel", rgba=(0.62, 0.64, 0.64, 1.0))
    aluminum = model.material("clear_anodized_aluminum", rgba=(0.78, 0.80, 0.78, 1.0))
    safety_blue = model.material("blue_height_marks", rgba=(0.05, 0.22, 0.72, 1.0))

    foot = model.part("foot")
    foot.visual(
        Cylinder(radius=0.32, length=0.055),
        origin=Origin(xyz=(0.0, 0.0, 0.0275)),
        material=black_paint,
        name="weighted_plate",
    )
    foot.visual(
        Box((0.78, 0.11, 0.04)),
        origin=Origin(xyz=(0.0, 0.0, 0.04)),
        material=dark_rubber,
        name="long_outrigger",
    )
    foot.visual(
        Box((0.11, 0.78, 0.04)),
        origin=Origin(xyz=(0.0, 0.0, 0.04)),
        material=dark_rubber,
        name="cross_outrigger",
    )
    foot.visual(
        Cylinder(radius=0.105, length=0.085),
        origin=Origin(xyz=(0.0, 0.0, 0.0975)),
        material=black_paint,
        name="base_collar",
    )
    foot.visual(
        mesh_from_cadquery(_square_tube(0.13, 0.096, 0.08, 1.05), "outer_sleeve"),
        material=brushed_steel,
        name="outer_sleeve",
    )
    foot.visual(
        mesh_from_cadquery(_square_tube(0.16, 0.096, 1.015, 1.07), "outer_clamp"),
        material=black_paint,
        name="outer_clamp",
    )
    foot.visual(
        Box((0.055, 0.038, 0.043)),
        origin=Origin(xyz=(0.095, 0.0, 1.043)),
        material=black_paint,
        name="outer_screw_boss",
    )
    foot.visual(
        Cylinder(radius=0.018, length=0.052),
        origin=Origin(xyz=(0.142, 0.0, 1.043), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_rubber,
        name="outer_thumb_pad",
    )
    for index, x in enumerate((-0.045, 0.045)):
        foot.visual(
            Box((0.006, 0.026, 0.060)),
            origin=Origin(xyz=(x, 0.0, 1.035)),
            material=dark_rubber,
            name=f"outer_guide_pad_{index}",
        )

    middle_sleeve = model.part("middle_sleeve")
    middle_sleeve.visual(
        mesh_from_cadquery(_square_tube(0.084, 0.060, -0.52, 0.50), "middle_tube"),
        material=aluminum,
        name="middle_tube",
    )
    middle_sleeve.visual(
        mesh_from_cadquery(_square_tube(0.108, 0.060, 0.470, 0.525), "middle_clamp"),
        material=black_paint,
        name="middle_clamp",
    )
    middle_sleeve.visual(
        Box((0.046, 0.032, 0.038)),
        origin=Origin(xyz=(0.068, 0.0, 0.498)),
        material=black_paint,
        name="middle_screw_boss",
    )
    middle_sleeve.visual(
        Cylinder(radius=0.014, length=0.044),
        origin=Origin(xyz=(0.107, 0.0, 0.498), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_rubber,
        name="middle_thumb_pad",
    )
    for index, x in enumerate((-0.0285, 0.0285)):
        middle_sleeve.visual(
            Box((0.003, 0.018, 0.050)),
            origin=Origin(xyz=(x, 0.0, 0.500)),
            material=dark_rubber,
            name=f"middle_guide_pad_{index}",
        )
    for index, z in enumerate((0.20, 0.28, 0.36, 0.44)):
        middle_sleeve.visual(
            Box((0.003, 0.032, 0.010)),
            origin=Origin(xyz=(0.0435, 0.0, z)),
            material=safety_blue,
            name=f"height_mark_{index}",
        )

    top_sleeve = model.part("top_sleeve")
    top_sleeve.visual(
        mesh_from_cadquery(_square_tube(0.054, 0.034, -0.46, 0.36), "top_tube"),
        material=brushed_steel,
        name="top_tube",
    )
    top_sleeve.visual(
        Box((0.080, 0.080, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, 0.3775)),
        material=black_paint,
        name="top_cap",
    )
    top_sleeve.visual(
        Box((0.020, 0.010, 0.22)),
        origin=Origin(xyz=(-0.032, 0.0, 0.25)),
        material=safety_blue,
        name="top_index_strip",
    )

    pan_platform = model.part("pan_platform")
    pan_platform.visual(
        Cylinder(radius=0.075, length=0.028),
        origin=Origin(xyz=(0.0, 0.0, 0.014)),
        material=black_paint,
        name="turntable_bearing",
    )
    pan_platform.visual(
        Box((0.22, 0.16, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, 0.037)),
        material=black_paint,
        name="fixture_plate",
    )
    for index, y in enumerate((-0.045, 0.045)):
        pan_platform.visual(
            Box((0.17, 0.018, 0.018)),
            origin=Origin(xyz=(0.0, y, 0.055)),
            material=aluminum,
            name=f"fixture_rail_{index}",
        )
    for index, (x, y) in enumerate(((-0.080, -0.058), (0.080, -0.058), (-0.080, 0.058), (0.080, 0.058))):
        pan_platform.visual(
            Cylinder(radius=0.008, length=0.006),
            origin=Origin(xyz=(x, y, 0.049)),
            material=brushed_steel,
            name=f"mount_bolt_{index}",
        )

    model.articulation(
        "foot_to_middle_sleeve",
        ArticulationType.PRISMATIC,
        parent=foot,
        child=middle_sleeve,
        origin=Origin(xyz=(0.0, 0.0, 1.05)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=0.0, upper=0.42, effort=140.0, velocity=0.18),
    )
    model.articulation(
        "middle_sleeve_to_top_sleeve",
        ArticulationType.PRISMATIC,
        parent=middle_sleeve,
        child=top_sleeve,
        origin=Origin(xyz=(0.0, 0.0, 0.50)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=0.0, upper=0.30, effort=90.0, velocity=0.16),
    )
    model.articulation(
        "top_sleeve_to_pan_platform",
        ArticulationType.REVOLUTE,
        parent=top_sleeve,
        child=pan_platform,
        origin=Origin(xyz=(0.0, 0.0, 0.395)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(lower=-math.pi, upper=math.pi, effort=8.0, velocity=1.2),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    foot = object_model.get_part("foot")
    middle = object_model.get_part("middle_sleeve")
    top = object_model.get_part("top_sleeve")
    pan = object_model.get_part("pan_platform")
    lift_1 = object_model.get_articulation("foot_to_middle_sleeve")
    lift_2 = object_model.get_articulation("middle_sleeve_to_top_sleeve")
    pan_joint = object_model.get_articulation("top_sleeve_to_pan_platform")

    ctx.expect_within(
        middle,
        foot,
        axes="xy",
        inner_elem="middle_tube",
        outer_elem="outer_sleeve",
        margin=0.002,
        name="middle sleeve stays centered in outer sleeve",
    )
    ctx.expect_overlap(
        middle,
        foot,
        axes="z",
        elem_a="middle_tube",
        elem_b="outer_sleeve",
        min_overlap=0.20,
        name="middle sleeve retains insertion when collapsed",
    )
    ctx.expect_within(
        top,
        middle,
        axes="xy",
        inner_elem="top_tube",
        outer_elem="middle_tube",
        margin=0.002,
        name="top sleeve stays centered in middle sleeve",
    )
    ctx.expect_overlap(
        top,
        middle,
        axes="z",
        elem_a="top_tube",
        elem_b="middle_tube",
        min_overlap=0.20,
        name="top sleeve retains insertion when collapsed",
    )
    ctx.expect_contact(
        pan,
        top,
        elem_a="turntable_bearing",
        elem_b="top_cap",
        contact_tol=0.001,
        name="pan bearing sits on the top cap",
    )

    middle_rest = ctx.part_world_position(middle)
    top_rest = ctx.part_world_position(top)
    pan_rest_aabb = ctx.part_world_aabb(pan)
    with ctx.pose({lift_1: 0.42, lift_2: 0.30, pan_joint: math.pi / 2.0}):
        ctx.expect_overlap(
            middle,
            foot,
            axes="z",
            elem_a="middle_tube",
            elem_b="outer_sleeve",
            min_overlap=0.08,
            name="middle sleeve remains captured at full lift",
        )
        ctx.expect_overlap(
            top,
            middle,
            axes="z",
            elem_a="top_tube",
            elem_b="middle_tube",
            min_overlap=0.12,
            name="top sleeve remains captured at full lift",
        )
        middle_extended = ctx.part_world_position(middle)
        top_extended = ctx.part_world_position(top)
        pan_rotated_aabb = ctx.part_world_aabb(pan)

    ctx.check(
        "prismatic lift stages extend upward",
        middle_rest is not None
        and top_rest is not None
        and middle_extended is not None
        and top_extended is not None
        and middle_extended[2] > middle_rest[2] + 0.40
        and top_extended[2] > top_rest[2] + 0.70,
        details=f"middle_rest={middle_rest}, middle_extended={middle_extended}, top_rest={top_rest}, top_extended={top_extended}",
    )
    if pan_rest_aabb is not None and pan_rotated_aabb is not None:
        rest_min, rest_max = pan_rest_aabb
        rot_min, rot_max = pan_rotated_aabb
        rest_dx = rest_max[0] - rest_min[0]
        rest_dy = rest_max[1] - rest_min[1]
        rot_dx = rot_max[0] - rot_min[0]
        rot_dy = rot_max[1] - rot_min[1]
    else:
        rest_dx = rest_dy = rot_dx = rot_dy = 0.0
    ctx.check(
        "pan platform rotates ninety degrees",
        rest_dx > rest_dy + 0.035 and rot_dy > rot_dx + 0.035,
        details=f"rest_dx={rest_dx}, rest_dy={rest_dy}, rot_dx={rot_dx}, rot_dy={rot_dy}",
    )

    return ctx.report()


object_model = build_object_model()
