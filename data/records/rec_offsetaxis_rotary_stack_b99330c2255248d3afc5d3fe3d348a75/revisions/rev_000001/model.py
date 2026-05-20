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
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="bridge_offset_rotary_fixture")

    cast_iron = Material("dark_cast_iron", color=(0.08, 0.085, 0.09, 1.0))
    machined = Material("machined_steel", color=(0.62, 0.64, 0.66, 1.0))
    bridge_paint = Material("fixture_blue", color=(0.05, 0.18, 0.34, 1.0))
    slot_black = Material("recess_black", color=(0.01, 0.012, 0.014, 1.0))
    brass = Material("oiled_bronze", color=(0.72, 0.52, 0.24, 1.0))

    base = model.part("base")
    base.visual(
        Cylinder(radius=0.39, length=0.08),
        origin=Origin(xyz=(0.0, 0.0, 0.04)),
        material=cast_iron,
        name="floor_plinth",
    )
    base.visual(
        Cylinder(radius=0.24, length=0.055),
        origin=Origin(xyz=(0.0, 0.0, 0.1075)),
        material=cast_iron,
        name="pedestal_step",
    )
    base.visual(
        Cylinder(radius=0.155, length=0.065),
        origin=Origin(xyz=(0.0, 0.0, 0.1675)),
        material=brass,
        name="base_bearing",
    )

    turntable = model.part("turntable")
    turntable.visual(
        Cylinder(radius=0.315, length=0.052),
        origin=Origin(xyz=(0.0, 0.0, 0.026)),
        material=machined,
        name="rotary_table",
    )
    turntable.visual(
        Cylinder(radius=0.13, length=0.125),
        origin=Origin(xyz=(0.0, 0.0, 0.085)),
        material=bridge_paint,
        name="central_boss",
    )
    turntable.visual(
        Box((0.66, 0.17, 0.055)),
        origin=Origin(xyz=(0.34, 0.0, 0.13)),
        material=bridge_paint,
        name="bridge_beam",
    )
    turntable.visual(
        Box((0.54, 0.035, 0.105)),
        origin=Origin(xyz=(0.36, -0.07, 0.092)),
        material=bridge_paint,
        name="front_web",
    )
    turntable.visual(
        Box((0.54, 0.035, 0.105)),
        origin=Origin(xyz=(0.36, 0.07, 0.092)),
        material=bridge_paint,
        name="rear_web",
    )
    turntable.visual(
        Cylinder(radius=0.135, length=0.145),
        origin=Origin(xyz=(0.68, 0.0, 0.1225)),
        material=bridge_paint,
        name="offset_column",
    )
    turntable.visual(
        Cylinder(radius=0.115, length=0.026),
        origin=Origin(xyz=(0.68, 0.0, 0.182)),
        material=brass,
        name="face_bearing",
    )

    faceplate = model.part("faceplate")
    faceplate.visual(
        Cylinder(radius=0.195, length=0.032),
        origin=Origin(xyz=(0.0, 0.0, 0.016)),
        material=machined,
        name="fixture_disk",
    )
    faceplate.visual(
        Cylinder(radius=0.067, length=0.052),
        origin=Origin(xyz=(0.0, 0.0, 0.057)),
        material=machined,
        name="center_hub",
    )
    faceplate.visual(
        Box((0.305, 0.030, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, 0.034)),
        material=slot_black,
        name="slot_x",
    )
    faceplate.visual(
        Box((0.030, 0.305, 0.004)),
        origin=Origin(xyz=(0.0, 0.0, 0.034)),
        material=slot_black,
        name="slot_y",
    )
    for i, (x, y) in enumerate(((0.105, 0.105), (-0.105, 0.105), (-0.105, -0.105), (0.105, -0.105))):
        faceplate.visual(
            Cylinder(radius=0.014, length=0.009),
            origin=Origin(xyz=(x, y, 0.0365)),
            material=slot_black,
            name=f"clamp_hole_{i}",
        )

    model.articulation(
        "base_axis",
        ArticulationType.REVOLUTE,
        parent=base,
        child=turntable,
        origin=Origin(xyz=(0.0, 0.0, 0.20)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=120.0,
            velocity=1.6,
            lower=-math.pi,
            upper=math.pi,
        ),
    )

    model.articulation(
        "faceplate_axis",
        ArticulationType.REVOLUTE,
        parent=turntable,
        child=faceplate,
        origin=Origin(xyz=(0.68, 0.0, 0.195)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(
            effort=35.0,
            velocity=2.4,
            lower=-math.pi,
            upper=math.pi,
        ),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    base = object_model.get_part("base")
    turntable = object_model.get_part("turntable")
    faceplate = object_model.get_part("faceplate")
    base_axis = object_model.get_articulation("base_axis")
    faceplate_axis = object_model.get_articulation("faceplate_axis")

    ctx.expect_contact(
        base,
        turntable,
        elem_a="base_bearing",
        elem_b="rotary_table",
        name="turntable rests on the fixed bearing",
    )
    ctx.expect_contact(
        turntable,
        faceplate,
        elem_a="face_bearing",
        elem_b="fixture_disk",
        name="offset faceplate is carried by its bridge bearing",
    )
    ctx.expect_origin_distance(
        turntable,
        faceplate,
        axes="xy",
        min_dist=0.65,
        max_dist=0.71,
        name="rotary axes are laterally offset",
    )
    ctx.check(
        "rotary axes are parallel vertical revolutes",
        base_axis.axis == (0.0, 0.0, 1.0)
        and faceplate_axis.axis == (0.0, 0.0, 1.0)
        and base_axis.articulation_type == ArticulationType.REVOLUTE
        and faceplate_axis.articulation_type == ArticulationType.REVOLUTE,
        details=f"base_axis={base_axis.axis}, faceplate_axis={faceplate_axis.axis}",
    )

    rest_position = ctx.part_world_position(faceplate)
    with ctx.pose({base_axis: 0.6, faceplate_axis: -0.9}):
        rotated_position = ctx.part_world_position(faceplate)
        ctx.expect_contact(
            turntable,
            faceplate,
            elem_a="face_bearing",
            elem_b="fixture_disk",
            name="faceplate stays seated while both axes rotate",
        )

    ctx.check(
        "base rotation carries the offset faceplate around the centerline",
        rest_position is not None
        and rotated_position is not None
        and rotated_position[0] < rest_position[0] - 0.05
        and rotated_position[1] > rest_position[1] + 0.30,
        details=f"rest={rest_position}, rotated={rotated_position}",
    )

    return ctx.report()


object_model = build_object_model()
