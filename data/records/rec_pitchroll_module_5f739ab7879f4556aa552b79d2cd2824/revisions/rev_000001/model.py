from __future__ import annotations

from math import pi

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
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_service_pitch_roll_head")

    dark_cast = Material("dark_cast_aluminum", rgba=(0.09, 0.10, 0.11, 1.0))
    black_trim = Material("black_rubber_trim", rgba=(0.015, 0.016, 0.018, 1.0))
    satin_steel = Material("satin_steel", rgba=(0.55, 0.57, 0.58, 1.0))
    cradle_paint = Material("cradle_graphite", rgba=(0.20, 0.22, 0.24, 1.0))
    glass = Material("smoked_sensor_glass", rgba=(0.02, 0.035, 0.05, 1.0))

    body = model.part("body")
    body.visual(
        Box((0.48, 0.36, 0.07)),
        origin=Origin(xyz=(0.0, 0.0, 0.035)),
        material=dark_cast,
        name="ground_plate",
    )
    body.visual(
        Box((0.27, 0.24, 0.11)),
        origin=Origin(xyz=(0.0, 0.0, 0.122)),
        material=dark_cast,
        name="lower_pedestal",
    )
    body.visual(
        Box((0.18, 0.16, 0.13)),
        origin=Origin(xyz=(0.0, 0.0, 0.238)),
        material=dark_cast,
        name="neck_block",
    )
    body.visual(
        Box((0.13, 0.13, 0.105)),
        origin=Origin(xyz=(0.0, 0.0, 0.334)),
        material=dark_cast,
        name="upper_stem",
    )
    body.visual(
        Box((0.20, 0.31, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, 0.39)),
        material=dark_cast,
        name="bearing_bridge",
    )
    body.visual(
        Box((0.20, 0.07, 0.24)),
        origin=Origin(xyz=(0.0, 0.135, 0.52)),
        material=dark_cast,
        name="front_bearing",
    )
    body.visual(
        Box((0.20, 0.07, 0.24)),
        origin=Origin(xyz=(0.0, -0.135, 0.52)),
        material=dark_cast,
        name="rear_bearing",
    )
    body.visual(
        Cylinder(radius=0.052, length=0.014),
        origin=Origin(xyz=(0.0, 0.176, 0.52), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=black_trim,
        name="front_bearing_cap",
    )
    body.visual(
        Cylinder(radius=0.052, length=0.014),
        origin=Origin(xyz=(0.0, -0.176, 0.52), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=black_trim,
        name="rear_bearing_cap",
    )

    outer_frame = model.part("outer_frame")
    outer_frame.visual(
        Cylinder(radius=0.026, length=0.204),
        origin=Origin(rpy=(-pi / 2.0, 0.0, 0.0)),
        material=satin_steel,
        name="roll_shaft",
    )
    outer_frame.visual(
        Box((0.315, 0.035, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, 0.0)),
        material=satin_steel,
        name="roll_web",
    )
    outer_frame.visual(
        Box((0.035, 0.070, 0.270)),
        origin=Origin(xyz=(0.145, 0.0, 0.0525)),
        material=satin_steel,
        name="side_rail_0",
    )
    outer_frame.visual(
        Box((0.035, 0.070, 0.270)),
        origin=Origin(xyz=(-0.145, 0.0, 0.0525)),
        material=satin_steel,
        name="side_rail_1",
    )
    outer_frame.visual(
        Box((0.325, 0.070, 0.035)),
        origin=Origin(xyz=(0.0, 0.0, 0.185)),
        material=satin_steel,
        name="top_crossbar",
    )
    outer_frame.visual(
        Cylinder(radius=0.030, length=0.045),
        origin=Origin(xyz=(0.145, 0.0, 0.095), rpy=(0.0, pi / 2.0, 0.0)),
        material=black_trim,
        name="pitch_bearing_0",
    )
    outer_frame.visual(
        Cylinder(radius=0.030, length=0.045),
        origin=Origin(xyz=(-0.145, 0.0, 0.095), rpy=(0.0, pi / 2.0, 0.0)),
        material=black_trim,
        name="pitch_bearing_1",
    )

    inner_cradle = model.part("inner_cradle")
    inner_cradle.visual(
        Cylinder(radius=0.018, length=0.252),
        origin=Origin(rpy=(0.0, pi / 2.0, 0.0)),
        material=satin_steel,
        name="pitch_axle",
    )
    inner_cradle.visual(
        Box((0.155, 0.100, 0.025)),
        origin=Origin(xyz=(0.0, 0.0, -0.045)),
        material=cradle_paint,
        name="cradle_floor",
    )
    inner_cradle.visual(
        Box((0.155, 0.020, 0.075)),
        origin=Origin(xyz=(0.0, -0.045, -0.005)),
        material=cradle_paint,
        name="rear_wall",
    )
    inner_cradle.visual(
        Box((0.155, 0.018, 0.070)),
        origin=Origin(xyz=(0.0, 0.046, -0.007)),
        material=cradle_paint,
        name="front_wall",
    )
    inner_cradle.visual(
        Box((0.018, 0.100, 0.078)),
        origin=Origin(xyz=(0.076, 0.0, -0.007)),
        material=cradle_paint,
        name="side_cheek_0",
    )
    inner_cradle.visual(
        Box((0.018, 0.100, 0.078)),
        origin=Origin(xyz=(-0.076, 0.0, -0.007)),
        material=cradle_paint,
        name="side_cheek_1",
    )
    inner_cradle.visual(
        Box((0.105, 0.006, 0.040)),
        origin=Origin(xyz=(0.0, 0.058, -0.004)),
        material=glass,
        name="front_window",
    )

    model.articulation(
        "roll",
        ArticulationType.REVOLUTE,
        parent=body,
        child=outer_frame,
        origin=Origin(xyz=(0.0, 0.0, 0.52)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=1.5, lower=-0.35, upper=0.35),
    )
    model.articulation(
        "pitch",
        ArticulationType.REVOLUTE,
        parent=outer_frame,
        child=inner_cradle,
        origin=Origin(xyz=(0.0, 0.0, 0.095)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=5.0, velocity=1.5, lower=-0.45, upper=0.45),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    body = object_model.get_part("body")
    outer_frame = object_model.get_part("outer_frame")
    inner_cradle = object_model.get_part("inner_cradle")
    roll = object_model.get_articulation("roll")
    pitch = object_model.get_articulation("pitch")

    ctx.allow_overlap(
        body,
        outer_frame,
        elem_a="front_bearing",
        elem_b="roll_shaft",
        reason="The roll shaft is intentionally captured a few millimeters inside the front bearing block.",
    )
    ctx.allow_overlap(
        body,
        outer_frame,
        elem_a="rear_bearing",
        elem_b="roll_shaft",
        reason="The roll shaft is intentionally captured a few millimeters inside the rear bearing block.",
    )
    ctx.allow_overlap(
        outer_frame,
        inner_cradle,
        elem_a="pitch_bearing_0",
        elem_b="pitch_axle",
        reason="The pitch axle is intentionally seated in the side bearing boss.",
    )
    ctx.allow_overlap(
        outer_frame,
        inner_cradle,
        elem_a="pitch_bearing_1",
        elem_b="pitch_axle",
        reason="The pitch axle is intentionally seated in the opposite side bearing boss.",
    )

    ctx.check(
        "two named revolute axes",
        roll.articulation_type == ArticulationType.REVOLUTE
        and pitch.articulation_type == ArticulationType.REVOLUTE
        and tuple(roll.axis) == (0.0, 1.0, 0.0)
        and tuple(pitch.axis) == (1.0, 0.0, 0.0),
        details=f"roll={roll.articulation_type}/{roll.axis}, pitch={pitch.articulation_type}/{pitch.axis}",
    )

    ctx.expect_gap(
        body,
        outer_frame,
        axis="y",
        positive_elem="front_bearing",
        negative_elem="roll_shaft",
        max_penetration=0.004,
        name="front roll shaft is locally seated",
    )
    ctx.expect_gap(
        outer_frame,
        body,
        axis="y",
        positive_elem="roll_shaft",
        negative_elem="rear_bearing",
        max_penetration=0.004,
        name="rear roll shaft is locally seated",
    )
    ctx.expect_gap(
        outer_frame,
        inner_cradle,
        axis="x",
        positive_elem="pitch_bearing_0",
        negative_elem="pitch_axle",
        max_penetration=0.005,
        name="positive pitch axle is locally seated",
    )
    ctx.expect_gap(
        inner_cradle,
        outer_frame,
        axis="x",
        positive_elem="pitch_axle",
        negative_elem="pitch_bearing_1",
        max_penetration=0.005,
        name="negative pitch axle is locally seated",
    )
    ctx.expect_within(
        inner_cradle,
        outer_frame,
        axes="xz",
        inner_elem="cradle_floor",
        outer_elem="top_crossbar",
        margin=0.160,
        name="cradle remains compact within outer frame width",
    )

    with ctx.pose({roll: 0.30, pitch: -0.35}):
        outer_aabb = ctx.part_world_aabb(outer_frame)
        body_aabb = ctx.part_world_aabb(body)
        ctx.check(
            "rolled frame stays above grounded body",
            outer_aabb is not None
            and body_aabb is not None
            and outer_aabb[0][2] > body_aabb[0][2] + 0.25,
            details=f"outer_aabb={outer_aabb}, body_aabb={body_aabb}",
        )

    return ctx.report()


object_model = build_object_model()
