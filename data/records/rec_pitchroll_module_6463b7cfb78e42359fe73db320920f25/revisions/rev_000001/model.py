from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    ExtrudeWithHolesGeometry,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
    mesh_from_geometry,
)


def _circle_profile(radius: float, segments: int = 64) -> list[tuple[float, float]]:
    return [
        (
            radius * math.cos(2.0 * math.pi * i / segments),
            radius * math.sin(2.0 * math.pi * i / segments),
        )
        for i in range(segments)
    ]


def _annular_plate(
    outer_radius: float,
    inner_radius: float,
    thickness: float,
    *,
    segments: int = 80,
) -> ExtrudeWithHolesGeometry:
    return ExtrudeWithHolesGeometry(
        _circle_profile(outer_radius, segments),
        [list(reversed(_circle_profile(inner_radius, segments)))],
        thickness,
        center=True,
    )


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="ring_cradle_pitch_roll_head")

    dark_steel = model.material("dark_steel", rgba=(0.06, 0.07, 0.075, 1.0))
    satin_steel = model.material("satin_steel", rgba=(0.48, 0.52, 0.54, 1.0))
    bearing_black = model.material("bearing_black", rgba=(0.015, 0.016, 0.018, 1.0))
    output_blue = model.material("output_blue", rgba=(0.12, 0.28, 0.62, 1.0))

    base = model.part("base_bracket")
    base.visual(
        Box((0.46, 0.32, 0.035)),
        origin=Origin(xyz=(-0.09, 0.0, 0.0175)),
        material=dark_steel,
        name="mount_foot",
    )
    base.visual(
        Box((0.13, 0.12, 0.245)),
        origin=Origin(xyz=(-0.115, 0.0, 0.1575)),
        material=dark_steel,
        name="upright_web",
    )
    base.visual(
        mesh_from_geometry(_annular_plate(0.064, 0.028, 0.105), "base_bearing_sleeve"),
        origin=Origin(xyz=(-0.115, 0.0, 0.34), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_steel,
        name="bearing_sleeve",
    )
    base.visual(
        Box((0.18, 0.018, 0.055)),
        origin=Origin(xyz=(-0.03, -0.151, 0.037)),
        material=satin_steel,
        name="mount_rail_0",
    )
    base.visual(
        Box((0.18, 0.018, 0.055)),
        origin=Origin(xyz=(-0.03, 0.151, 0.037)),
        material=satin_steel,
        name="mount_rail_1",
    )

    roll = model.part("roll_frame")
    roll.visual(
        mesh_from_geometry(_annular_plate(0.218, 0.162, 0.040), "roll_outer_ring"),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_steel,
        name="outer_ring",
    )
    roll.visual(
        Cylinder(radius=0.025, length=0.215),
        origin=Origin(xyz=(-0.128, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="roll_shaft",
    )
    roll.visual(
        Cylinder(radius=0.050, length=0.055),
        origin=Origin(xyz=(-0.035, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_steel,
        name="roll_hub",
    )
    roll.visual(
        Box((0.032, 0.030, 0.285)),
        origin=Origin(xyz=(-0.006, 0.0, 0.0)),
        material=satin_steel,
        name="vertical_spoke",
    )
    roll.visual(
        mesh_from_geometry(_annular_plate(0.043, 0.021, 0.070), "pitch_bearing_0"),
        origin=Origin(xyz=(0.0, -0.184, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=satin_steel,
        name="pitch_bearing_0",
    )
    roll.visual(
        mesh_from_geometry(_annular_plate(0.043, 0.021, 0.070), "pitch_bearing_1"),
        origin=Origin(xyz=(0.0, 0.184, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=satin_steel,
        name="pitch_bearing_1",
    )
    roll.visual(
        Cylinder(radius=0.049, length=0.010),
        origin=Origin(xyz=(-0.232, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=bearing_black,
        name="rear_retainer",
    )

    cradle = model.part("pitch_cradle")
    cradle.visual(
        Box((0.160, 0.050, 0.065)),
        origin=Origin(xyz=(0.098, -0.105, 0.0)),
        material=dark_steel,
        name="side_arm_0",
    )
    cradle.visual(
        Box((0.160, 0.050, 0.065)),
        origin=Origin(xyz=(0.098, 0.105, 0.0)),
        material=dark_steel,
        name="side_arm_1",
    )
    cradle.visual(
        Box((0.040, 0.205, 0.105)),
        origin=Origin(xyz=(0.175, 0.0, 0.0)),
        material=dark_steel,
        name="front_bridge",
    )
    cradle.visual(
        Box((0.014, 0.130, 0.082)),
        origin=Origin(xyz=(0.200, 0.0, 0.0)),
        material=output_blue,
        name="output_face",
    )
    cradle.visual(
        Box((0.006, 0.082, 0.052)),
        origin=Origin(xyz=(0.208, 0.0, 0.0)),
        material=bearing_black,
        name="output_insert",
    )
    cradle.visual(
        Cylinder(radius=0.018, length=0.104),
        origin=Origin(xyz=(0.0, -0.162, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="pitch_trunnion_0",
    )
    cradle.visual(
        Cylinder(radius=0.018, length=0.104),
        origin=Origin(xyz=(0.0, 0.162, 0.0), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="pitch_trunnion_1",
    )

    model.articulation(
        "roll_joint",
        ArticulationType.REVOLUTE,
        parent=base,
        child=roll,
        origin=Origin(xyz=(0.0, 0.0, 0.34)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=2.0, lower=-1.57, upper=1.57),
    )
    model.articulation(
        "pitch_joint",
        ArticulationType.REVOLUTE,
        parent=roll,
        child=cradle,
        origin=Origin(),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.4, lower=-0.85, upper=0.85),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base_bracket")
    roll = object_model.get_part("roll_frame")
    cradle = object_model.get_part("pitch_cradle")
    roll_joint = object_model.get_articulation("roll_joint")
    pitch_joint = object_model.get_articulation("pitch_joint")

    ctx.allow_overlap(
        base,
        roll,
        elem_a="bearing_sleeve",
        elem_b="roll_shaft",
        reason=(
            "The roll shaft is intentionally captured inside the base bearing sleeve; "
            "the mesh collision proxy treats the visible sleeve bore as filled."
        ),
    )
    ctx.allow_overlap(
        cradle,
        roll,
        elem_a="pitch_trunnion_0",
        elem_b="pitch_bearing_0",
        reason=(
            "The pitch trunnion is intentionally seated in its bearing bore; "
            "the mesh collision proxy treats the bore as a solid sleeve."
        ),
    )
    ctx.allow_overlap(
        cradle,
        roll,
        elem_a="pitch_trunnion_1",
        elem_b="pitch_bearing_1",
        reason=(
            "The pitch trunnion is intentionally seated in its bearing bore; "
            "the mesh collision proxy treats the bore as a solid sleeve."
        ),
    )
    ctx.allow_overlap(
        cradle,
        roll,
        elem_a="pitch_trunnion_0",
        elem_b="outer_ring",
        reason=(
            "The side trunnion passes through a bored lug at the roll ring; "
            "the simplified annular frame leaves that local through-bore implicit."
        ),
    )
    ctx.allow_overlap(
        cradle,
        roll,
        elem_a="pitch_trunnion_1",
        elem_b="outer_ring",
        reason=(
            "The side trunnion passes through a bored lug at the roll ring; "
            "the simplified annular frame leaves that local through-bore implicit."
        ),
    )

    ctx.expect_within(
        roll,
        base,
        axes="yz",
        inner_elem="roll_shaft",
        outer_elem="bearing_sleeve",
        margin=0.004,
        name="roll shaft is captured by the base sleeve",
    )
    ctx.expect_overlap(
        roll,
        base,
        axes="x",
        elem_a="roll_shaft",
        elem_b="bearing_sleeve",
        min_overlap=0.08,
        name="roll shaft remains inserted through bearing",
    )
    ctx.expect_within(
        cradle,
        roll,
        axes="xz",
        inner_elem="pitch_trunnion_0",
        outer_elem="pitch_bearing_0",
        margin=0.004,
        name="pitch trunnion 0 is centered in its bearing",
    )
    ctx.expect_within(
        cradle,
        roll,
        axes="xz",
        inner_elem="pitch_trunnion_1",
        outer_elem="pitch_bearing_1",
        margin=0.004,
        name="pitch trunnion 1 is centered in its bearing",
    )
    ctx.expect_overlap(
        cradle,
        roll,
        axes="y",
        elem_a="pitch_trunnion_0",
        elem_b="pitch_bearing_0",
        min_overlap=0.045,
        name="pitch trunnion 0 stays inserted",
    )
    ctx.expect_overlap(
        cradle,
        roll,
        axes="y",
        elem_a="pitch_trunnion_1",
        elem_b="pitch_bearing_1",
        min_overlap=0.045,
        name="pitch trunnion 1 stays inserted",
    )

    rest_face = ctx.part_element_world_aabb(cradle, elem="output_face")
    with ctx.pose({pitch_joint: 0.65}):
        pitched_face = ctx.part_element_world_aabb(cradle, elem="output_face")
    ctx.check(
        "positive pitch lifts the output face",
        rest_face is not None
        and pitched_face is not None
        and pitched_face[1][2] > rest_face[1][2] + 0.035,
        details=f"rest={rest_face}, pitched={pitched_face}",
    )

    rest_cradle = ctx.part_world_position(cradle)
    with ctx.pose({roll_joint: 0.75}):
        rolled_cradle = ctx.part_world_position(cradle)
    ctx.check(
        "roll joint keeps cradle on the main axis",
        rest_cradle is not None
        and rolled_cradle is not None
        and abs(rolled_cradle[0] - rest_cradle[0]) < 1e-6,
        details=f"rest={rest_cradle}, rolled={rolled_cradle}",
    )

    return ctx.report()


object_model = build_object_model()
