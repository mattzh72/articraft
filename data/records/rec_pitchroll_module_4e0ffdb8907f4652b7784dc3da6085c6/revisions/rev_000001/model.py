from __future__ import annotations

import math

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Cylinder,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="tower_pitch_roll_unit")

    painted_casting = model.material("painted_casting", rgba=(0.08, 0.10, 0.12, 1.0))
    dark_bearing = model.material("dark_bearing", rgba=(0.015, 0.016, 0.018, 1.0))
    brushed_steel = model.material("brushed_steel", rgba=(0.62, 0.64, 0.62, 1.0))
    output_blue = model.material("output_blue", rgba=(0.05, 0.18, 0.35, 1.0))

    head_z = 0.43

    pedestal = model.part("pedestal")
    pedestal.visual(
        Cylinder(radius=0.18, length=0.035),
        origin=Origin(xyz=(0.0, 0.0, 0.0175)),
        material=painted_casting,
        name="base_plate",
    )
    pedestal.visual(
        Cylinder(radius=0.055, length=0.300),
        origin=Origin(xyz=(0.0, 0.0, 0.185)),
        material=painted_casting,
        name="tower_post",
    )
    pedestal.visual(
        Cylinder(radius=0.095, length=0.028),
        origin=Origin(xyz=(0.0, 0.0, 0.349)),
        material=painted_casting,
        name="top_flange",
    )
    pedestal.visual(
        Box((0.070, 0.115, 0.072)),
        origin=Origin(xyz=(0.0, 0.0, head_z - 0.066)),
        material=painted_casting,
        name="bearing_stem",
    )
    pedestal.visual(
        Cylinder(radius=0.055, length=0.105),
        origin=Origin(xyz=(0.0, 0.0, head_z), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=dark_bearing,
        name="roll_bearing",
    )

    roll_frame = model.part("roll_frame")
    roll_frame.visual(
        Cylinder(radius=0.026, length=0.154),
        origin=Origin(xyz=(0.018, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brushed_steel,
        name="roll_shaft",
    )
    roll_frame.visual(
        Cylinder(radius=0.041, length=0.014),
        origin=Origin(xyz=(-0.066, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brushed_steel,
        name="shaft_collar",
    )
    roll_frame.visual(
        Box((0.014, 0.032, 0.120)),
        origin=Origin(xyz=(0.092, 0.0, 0.040)),
        material=painted_casting,
        name="roll_web",
    )
    roll_frame.visual(
        Box((0.055, 0.300, 0.030)),
        origin=Origin(xyz=(0.120, 0.0, 0.105)),
        material=painted_casting,
        name="top_bar",
    )
    roll_frame.visual(
        Box((0.055, 0.300, 0.030)),
        origin=Origin(xyz=(0.120, 0.0, -0.105)),
        material=painted_casting,
        name="bottom_bar",
    )
    for index, side_y in enumerate((-0.134, 0.134)):
        roll_frame.visual(
            Box((0.055, 0.032, 0.078)),
            origin=Origin(xyz=(0.120, side_y, 0.068)),
            material=painted_casting,
            name=f"side_upper_{index}",
        )
        roll_frame.visual(
            Box((0.055, 0.032, 0.078)),
            origin=Origin(xyz=(0.120, side_y, -0.068)),
            material=painted_casting,
            name=f"side_lower_{index}",
        )
        roll_frame.visual(
            Cylinder(radius=0.032, length=0.052),
            origin=Origin(xyz=(0.120, side_y, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=dark_bearing,
            name=f"pitch_boss_{index}",
        )

    pitch_cradle = model.part("pitch_cradle")
    pitch_cradle.visual(
        Cylinder(radius=0.014, length=0.312),
        origin=Origin(rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=brushed_steel,
        name="pitch_axle",
    )
    pitch_cradle.visual(
        Box((0.070, 0.138, 0.072)),
        origin=Origin(xyz=(0.046, 0.0, 0.0)),
        material=output_blue,
        name="cradle_body",
    )
    pitch_cradle.visual(
        Box((0.024, 0.018, 0.106)),
        origin=Origin(xyz=(0.040, -0.070, 0.0)),
        material=output_blue,
        name="cradle_side_0",
    )
    pitch_cradle.visual(
        Box((0.024, 0.018, 0.106)),
        origin=Origin(xyz=(0.040, 0.070, 0.0)),
        material=output_blue,
        name="cradle_side_1",
    )
    pitch_cradle.visual(
        Box((0.014, 0.098, 0.052)),
        origin=Origin(xyz=(0.087, 0.0, 0.0)),
        material=dark_bearing,
        name="output_face",
    )

    model.articulation(
        "roll_axis",
        ArticulationType.REVOLUTE,
        parent=pedestal,
        child=roll_frame,
        origin=Origin(xyz=(0.0, 0.0, head_z)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=18.0, velocity=2.0, lower=-0.75, upper=0.75),
    )
    model.articulation(
        "pitch_axis",
        ArticulationType.REVOLUTE,
        parent=roll_frame,
        child=pitch_cradle,
        origin=Origin(xyz=(0.120, 0.0, 0.0)),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=10.0, velocity=2.5, lower=-0.55, upper=0.55),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    pedestal = object_model.get_part("pedestal")
    roll_frame = object_model.get_part("roll_frame")
    pitch_cradle = object_model.get_part("pitch_cradle")
    roll = object_model.get_articulation("roll_axis")
    pitch = object_model.get_articulation("pitch_axis")

    ctx.allow_overlap(
        pedestal,
        roll_frame,
        elem_a="roll_bearing",
        elem_b="roll_shaft",
        reason="The roll shaft is intentionally captured inside the tower bearing housing.",
    )
    ctx.expect_within(
        roll_frame,
        pedestal,
        axes="yz",
        inner_elem="roll_shaft",
        outer_elem="roll_bearing",
        margin=0.0,
        name="roll shaft centered in bearing",
    )
    ctx.expect_overlap(
        pedestal,
        roll_frame,
        axes="x",
        elem_a="roll_bearing",
        elem_b="roll_shaft",
        min_overlap=0.070,
        name="roll shaft retained in bearing",
    )

    for index in (0, 1):
        ctx.allow_overlap(
            roll_frame,
            pitch_cradle,
            elem_a=f"pitch_boss_{index}",
            elem_b="pitch_axle",
            reason="The pitch axle is intentionally captured through the roll-frame trunnion boss.",
        )
        ctx.expect_within(
            pitch_cradle,
            roll_frame,
            axes="xz",
            inner_elem="pitch_axle",
            outer_elem=f"pitch_boss_{index}",
            margin=0.0,
            name=f"pitch axle centered in boss {index}",
        )
        ctx.expect_overlap(
            roll_frame,
            pitch_cradle,
            axes="y",
            elem_a=f"pitch_boss_{index}",
            elem_b="pitch_axle",
            min_overlap=0.040,
            name=f"pitch axle retained in boss {index}",
        )

    ctx.expect_within(
        pitch_cradle,
        roll_frame,
        axes="yz",
        inner_elem="cradle_body",
        margin=0.0,
        name="inner output cradle fits inside outer member",
    )

    dot = sum(a * b for a, b in zip(roll.axis, pitch.axis))
    ctx.check(
        "roll and pitch axes are perpendicular",
        abs(dot) < 1.0e-6,
        details=f"roll axis={roll.axis}, pitch axis={pitch.axis}",
    )

    outer_bb = ctx.part_world_aabb(roll_frame)
    cradle_bb = ctx.part_element_world_aabb(pitch_cradle, elem="cradle_body")
    if outer_bb is not None and cradle_bb is not None:
        outer_y = outer_bb[1][1] - outer_bb[0][1]
        outer_z = outer_bb[1][2] - outer_bb[0][2]
        cradle_y = cradle_bb[1][1] - cradle_bb[0][1]
        cradle_z = cradle_bb[1][2] - cradle_bb[0][2]
        larger = outer_y > cradle_y + 0.10 and outer_z > cradle_z + 0.10
    else:
        larger = False
    ctx.check(
        "outer roll member visibly larger than inner cradle",
        larger,
        details=f"outer_bb={outer_bb}, cradle_bb={cradle_bb}",
    )

    def elem_center_z(part, elem):
        bb = ctx.part_element_world_aabb(part, elem=elem)
        if bb is None:
            return None
        return 0.5 * (bb[0][2] + bb[1][2])

    def elem_center_y(part, elem):
        bb = ctx.part_element_world_aabb(part, elem=elem)
        if bb is None:
            return None
        return 0.5 * (bb[0][1] + bb[1][1])

    top_rest_y = elem_center_y(roll_frame, "top_bar")
    with ctx.pose({roll: 0.55}):
        top_rolled_y = elem_center_y(roll_frame, "top_bar")
        ctx.expect_within(
            pitch_cradle,
            roll_frame,
            axes="yz",
            inner_elem="cradle_body",
            margin=0.010,
            name="cradle remains nested during roll",
        )
    ctx.check(
        "roll joint rotates the outer member",
        top_rest_y is not None
        and top_rolled_y is not None
        and abs(top_rolled_y - top_rest_y) > 0.030,
        details=f"rest_y={top_rest_y}, rolled_y={top_rolled_y}",
    )

    face_rest_z = elem_center_z(pitch_cradle, "output_face")
    with ctx.pose({pitch: 0.45}):
        face_pitched_z = elem_center_z(pitch_cradle, "output_face")
        ctx.expect_within(
            pitch_cradle,
            roll_frame,
            axes="yz",
            inner_elem="cradle_body",
            margin=0.010,
            name="cradle remains inside frame while pitching",
        )
    ctx.check(
        "pitch joint tilts the inner output cradle",
        face_rest_z is not None
        and face_pitched_z is not None
        and abs(face_pitched_z - face_rest_z) > 0.020,
        details=f"rest_z={face_rest_z}, pitched_z={face_pitched_z}",
    )

    return ctx.report()


object_model = build_object_model()
