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


def _leaf_plate(width: float, length: float, thickness: float, center_x: float) -> cq.Workplane:
    """Perforated rectangular hinge leaf, authored in local hinge coordinates."""
    hole_points = [(-0.025, -0.095), (0.025, -0.095), (-0.025, 0.095), (0.025, 0.095)]
    return (
        cq.Workplane("XY")
        .box(width, length, thickness)
        .edges("|Z")
        .fillet(0.004)
        .faces(">Z")
        .workplane()
        .pushPoints(hole_points)
        .hole(0.013)
        .translate((center_x, 0.0, 0.0))
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="heavy_duty_hinge")

    steel = Material("brushed_steel", rgba=(0.58, 0.60, 0.58, 1.0))
    dark_steel = Material("dark_pin_steel", rgba=(0.08, 0.075, 0.07, 1.0))
    block_metal = Material("dark_mount_block", rgba=(0.20, 0.22, 0.24, 1.0))
    screw_black = Material("black_socket_screws", rgba=(0.015, 0.015, 0.014, 1.0))

    fixed = model.part("fixed_leaf")
    fixed.visual(
        Box((0.12, 0.34, 0.040)),
        origin=Origin(xyz=(-0.090, 0.0, -0.025)),
        material=block_metal,
        name="mount_block",
    )
    fixed.visual(
        mesh_from_cadquery(_leaf_plate(0.105, 0.300, 0.010, -0.0775), "fixed_leaf_plate"),
        material=steel,
        name="leaf_plate",
    )
    fixed.visual(
        Cylinder(radius=0.006, length=0.330),
        origin=Origin(xyz=(0.0, 0.0, 0.006), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="central_pin",
    )

    for index, (y, length) in enumerate(((-0.125, 0.050), (0.0, 0.050), (0.125, 0.050))):
        fixed.visual(
            Cylinder(radius=0.018, length=length),
            origin=Origin(xyz=(0.0, y, 0.006), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=steel,
            name=f"fixed_knuckle_{index}",
        )
        fixed.visual(
            Box((0.024, length, 0.014)),
            origin=Origin(xyz=(-0.024, y, 0.004)),
            material=steel,
            name=f"fixed_web_{index}",
        )

    for index, (x, y) in enumerate(
        ((-0.1025, -0.095), (-0.0525, -0.095), (-0.1025, 0.095), (-0.0525, 0.095))
    ):
        fixed.visual(
            Cylinder(radius=0.010, length=0.003),
            origin=Origin(xyz=(x, y, 0.0065)),
            material=screw_black,
            name=f"fixed_screw_{index}",
        )

    swing = model.part("swing_leaf")
    swing.visual(
        mesh_from_cadquery(_leaf_plate(0.105, 0.300, 0.010, 0.0775), "swing_leaf_plate"),
        material=steel,
        name="leaf_plate",
    )

    for index, (y, length) in enumerate(((-0.0625, 0.065), (0.0625, 0.065))):
        swing.visual(
            Cylinder(radius=0.018, length=length),
            origin=Origin(xyz=(0.0, y, 0.006), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=steel,
            name=f"swing_knuckle_{index}",
        )
        swing.visual(
            Box((0.024, length, 0.014)),
            origin=Origin(xyz=(0.024, y, 0.004)),
            material=steel,
            name=f"swing_web_{index}",
        )

    for index, (x, y) in enumerate(
        ((0.0525, -0.095), (0.1025, -0.095), (0.0525, 0.095), (0.1025, 0.095))
    ):
        swing.visual(
            Cylinder(radius=0.010, length=0.003),
            origin=Origin(xyz=(x, y, 0.0065)),
            material=screw_black,
            name=f"swing_screw_{index}",
        )

    model.articulation(
        "leaf_hinge",
        ArticulationType.REVOLUTE,
        parent=fixed,
        child=swing,
        origin=Origin(xyz=(0.0, 0.0, 0.006)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=120.0, velocity=2.0, lower=0.0, upper=math.pi),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    fixed = object_model.get_part("fixed_leaf")
    swing = object_model.get_part("swing_leaf")
    hinge = object_model.get_articulation("leaf_hinge")

    for index in range(2):
        ctx.allow_overlap(
            fixed,
            swing,
            elem_a="central_pin",
            elem_b=f"swing_knuckle_{index}",
            reason="The hinge pin is intentionally captured inside the rotating knuckle bore.",
        )
        ctx.expect_within(
            fixed,
            swing,
            axes="xz",
            inner_elem="central_pin",
            outer_elem=f"swing_knuckle_{index}",
            margin=0.013,
            name=f"pin remains centered in swing knuckle {index}",
        )
        ctx.expect_overlap(
            fixed,
            swing,
            axes="y",
            elem_a="central_pin",
            elem_b=f"swing_knuckle_{index}",
            min_overlap=0.060,
            name=f"pin spans swing knuckle {index}",
        )

    ctx.expect_gap(
        swing,
        fixed,
        axis="x",
        positive_elem="leaf_plate",
        negative_elem="leaf_plate",
        min_gap=0.040,
        max_gap=0.060,
        name="closed leaves flank the pin with a realistic gap",
    )

    rest_pos = ctx.part_world_position(swing)
    with ctx.pose({hinge: math.pi / 2.0}):
        rotated_pos = ctx.part_world_position(swing)
        ctx.expect_overlap(
            fixed,
            swing,
            axes="y",
            elem_a="central_pin",
            elem_b="swing_knuckle_0",
            min_overlap=0.060,
            name="quarter turn retains the lower rotating knuckle on the pin",
        )

    limits = hinge.motion_limits
    ctx.check(
        "hinge travel is about 180 degrees",
        limits is not None
        and limits.lower is not None
        and limits.upper is not None
        and abs(limits.lower) < 1e-6
        and abs(limits.upper - math.pi) < 1e-6,
        details=f"limits={limits}",
    )
    ctx.check(
        "revolute child frame stays on pin axis",
        rest_pos is not None and rotated_pos is not None and abs(rest_pos[1] - rotated_pos[1]) < 1e-6,
        details=f"rest={rest_pos}, rotated={rotated_pos}",
    )

    return ctx.report()


object_model = build_object_model()
