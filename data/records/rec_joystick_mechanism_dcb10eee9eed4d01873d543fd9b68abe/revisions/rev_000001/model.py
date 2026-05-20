from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    Material,
    MotionLimits,
    Origin,
    Sphere,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="under_slung_joystick")

    satin_aluminum = model.material("satin_aluminum", rgba=(0.62, 0.64, 0.63, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.08, 0.085, 0.09, 1.0))
    black_rubber = model.material("black_rubber", rgba=(0.015, 0.014, 0.012, 1.0))
    bolt_black = model.material("blackened_bolts", rgba=(0.02, 0.02, 0.018, 1.0))

    top_bracket = model.part("top_bracket")
    top_bracket.visual(
        Box((0.210, 0.145, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, 0.113)),
        material=satin_aluminum,
        name="mounting_plate",
    )
    top_bracket.visual(
        Box((0.125, 0.082, 0.016)),
        origin=Origin(xyz=(0.0, 0.0, 0.127)),
        material=satin_aluminum,
        name="raised_boss",
    )
    top_bracket.visual(
        Box((0.022, 0.060, 0.134)),
        origin=Origin(xyz=(-0.080, 0.0, 0.040)),
        material=satin_aluminum,
        name="bearing_block_0",
    )
    top_bracket.visual(
        Cylinder(radius=0.024, length=0.010),
        origin=Origin(xyz=(-0.096, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_aluminum,
        name="bearing_boss_0",
    )
    top_bracket.visual(
        Box((0.022, 0.060, 0.134)),
        origin=Origin(xyz=(0.080, 0.0, 0.040)),
        material=satin_aluminum,
        name="bearing_block_1",
    )
    top_bracket.visual(
        Cylinder(radius=0.024, length=0.010),
        origin=Origin(xyz=(0.096, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=satin_aluminum,
        name="bearing_boss_1",
    )
    for idx, (x, y) in enumerate(
        ((-0.075, -0.048), (-0.075, 0.048), (0.075, -0.048), (0.075, 0.048))
    ):
        top_bracket.visual(
            Cylinder(radius=0.008, length=0.006),
            origin=Origin(xyz=(x, y, 0.122)),
            material=bolt_black,
            name=f"mount_bolt_{idx}",
        )

    outer_yoke = model.part("outer_yoke")
    outer_yoke.visual(
        Cylinder(radius=0.010, length=0.138),
        origin=Origin(rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_steel,
        name="outer_trunnion",
    )
    outer_yoke.visual(
        Box((0.050, 0.030, 0.032)),
        origin=Origin(xyz=(0.0, 0.0, -0.019)),
        material=satin_aluminum,
        name="outer_neck",
    )
    outer_yoke.visual(
        Box((0.050, 0.126, 0.022)),
        origin=Origin(xyz=(0.0, 0.0, -0.039)),
        material=satin_aluminum,
        name="outer_bridge",
    )
    outer_yoke.visual(
        Box((0.040, 0.016, 0.096)),
        origin=Origin(xyz=(0.0, -0.056, -0.087)),
        material=satin_aluminum,
        name="outer_cheek_0",
    )
    outer_yoke.visual(
        Cylinder(radius=0.019, length=0.010),
        origin=Origin(xyz=(0.0, -0.069, -0.078), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=satin_aluminum,
        name="inner_bearing_boss_0",
    )
    outer_yoke.visual(
        Box((0.040, 0.016, 0.096)),
        origin=Origin(xyz=(0.0, 0.056, -0.087)),
        material=satin_aluminum,
        name="outer_cheek_1",
    )
    outer_yoke.visual(
        Cylinder(radius=0.019, length=0.010),
        origin=Origin(xyz=(0.0, 0.069, -0.078), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=satin_aluminum,
        name="inner_bearing_boss_1",
    )
    outer_yoke.visual(
        Box((0.020, 0.100, 0.016)),
        origin=Origin(xyz=(0.0, 0.0, -0.125)),
        material=satin_aluminum,
        name="stop_tie",
    )

    inner_yoke = model.part("inner_yoke")
    inner_yoke.visual(
        Cylinder(radius=0.008, length=0.096),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=dark_steel,
        name="inner_trunnion",
    )
    inner_yoke.visual(
        Cylinder(radius=0.021, length=0.046),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=satin_aluminum,
        name="pivot_boss",
    )
    for idx, x in enumerate((-0.020, 0.020)):
        inner_yoke.visual(
            Box((0.012, 0.030, 0.067)),
            origin=Origin(xyz=(x, 0.0, -0.044)),
            material=satin_aluminum,
            name=f"clamp_cheek_{idx}",
        )
    inner_yoke.visual(
        Box((0.058, 0.030, 0.018)),
        origin=Origin(xyz=(0.0, 0.0, -0.086)),
        material=satin_aluminum,
        name="stick_socket",
    )
    inner_yoke.visual(
        Cylinder(radius=0.020, length=0.018),
        origin=Origin(xyz=(0.0, 0.0, -0.095)),
        material=satin_aluminum,
        name="socket_collar",
    )

    stick = model.part("stick")
    stick.visual(
        Cylinder(radius=0.015, length=0.026),
        origin=Origin(xyz=(0.0, 0.0, -0.013)),
        material=dark_steel,
        name="stick_mount",
    )
    stick.visual(
        Cylinder(radius=0.0085, length=0.175),
        origin=Origin(xyz=(0.0, 0.0, -0.106)),
        material=dark_steel,
        name="stick_shaft",
    )
    stick.visual(
        Cylinder(radius=0.017, length=0.070),
        origin=Origin(xyz=(0.0, 0.0, -0.215)),
        material=black_rubber,
        name="grip_sleeve",
    )
    stick.visual(
        Sphere(radius=0.021),
        origin=Origin(xyz=(0.0, 0.0, -0.252)),
        material=black_rubber,
        name="rounded_grip_end",
    )

    model.articulation(
        "bracket_to_outer",
        ArticulationType.REVOLUTE,
        parent=top_bracket,
        child=outer_yoke,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=7.0, velocity=4.0, lower=-0.55, upper=0.55),
    )
    model.articulation(
        "outer_to_inner",
        ArticulationType.REVOLUTE,
        parent=outer_yoke,
        child=inner_yoke,
        origin=Origin(xyz=(0.0, 0.0, -0.078)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=6.0, velocity=4.0, lower=-0.50, upper=0.50),
    )
    model.articulation(
        "inner_to_stick",
        ArticulationType.FIXED,
        parent=inner_yoke,
        child=stick,
        origin=Origin(xyz=(0.0, 0.0, -0.104)),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    top_bracket = object_model.get_part("top_bracket")
    outer_yoke = object_model.get_part("outer_yoke")
    inner_yoke = object_model.get_part("inner_yoke")
    stick = object_model.get_part("stick")
    bracket_to_outer = object_model.get_articulation("bracket_to_outer")
    outer_to_inner = object_model.get_articulation("outer_to_inner")

    ctx.check(
        "two orthogonal revolute gimbal axes",
        bracket_to_outer.articulation_type == ArticulationType.REVOLUTE
        and outer_to_inner.articulation_type == ArticulationType.REVOLUTE
        and abs(sum(a * b for a, b in zip(bracket_to_outer.axis, outer_to_inner.axis))) < 1e-6,
        details=f"axes={bracket_to_outer.axis}, {outer_to_inner.axis}",
    )
    ctx.expect_contact(
        top_bracket,
        outer_yoke,
        elem_a="bearing_block_0",
        elem_b="outer_trunnion",
        contact_tol=0.001,
        name="outer trunnion bears on one bracket cheek",
    )
    ctx.expect_contact(
        top_bracket,
        outer_yoke,
        elem_a="bearing_block_1",
        elem_b="outer_trunnion",
        contact_tol=0.001,
        name="outer trunnion bears on opposite bracket cheek",
    )
    ctx.expect_contact(
        outer_yoke,
        inner_yoke,
        elem_a="outer_cheek_0",
        elem_b="inner_trunnion",
        contact_tol=0.001,
        name="inner trunnion bears on one outer yoke cheek",
    )
    ctx.expect_contact(
        outer_yoke,
        inner_yoke,
        elem_a="outer_cheek_1",
        elem_b="inner_trunnion",
        contact_tol=0.001,
        name="inner trunnion bears on opposite outer yoke cheek",
    )
    ctx.expect_contact(
        inner_yoke,
        stick,
        elem_a="socket_collar",
        elem_b="stick_mount",
        contact_tol=0.001,
        name="stick mount seats against inner yoke socket",
    )
    ctx.expect_gap(
        top_bracket,
        stick,
        axis="z",
        min_gap=0.11,
        name="stick hangs below the top bracket",
    )

    rest_pos = ctx.part_world_position(stick)
    with ctx.pose({bracket_to_outer: 0.45}):
        pitch_pos = ctx.part_world_position(stick)
    with ctx.pose({outer_to_inner: 0.40}):
        roll_pos = ctx.part_world_position(stick)
    ctx.check(
        "outer yoke pitch swings the stick fore-aft",
        rest_pos is not None and pitch_pos is not None and abs(pitch_pos[1] - rest_pos[1]) > 0.035,
        details=f"rest={rest_pos}, pitched={pitch_pos}",
    )
    ctx.check(
        "inner yoke roll swings the stick side-to-side",
        rest_pos is not None and roll_pos is not None and abs(roll_pos[0] - rest_pos[0]) > 0.030,
        details=f"rest={rest_pos}, rolled={roll_pos}",
    )

    return ctx.report()


object_model = build_object_model()
