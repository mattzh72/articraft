from __future__ import annotations

from math import pi

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
    model = ArticulatedObject(name="low_profile_pitch_roll_gimbal")

    dark = model.material("dark_anodized_aluminum", color=(0.08, 0.085, 0.09, 1.0))
    outer_metal = model.material("satin_outer_frame", color=(0.42, 0.46, 0.48, 1.0))
    inner_metal = model.material("brushed_inner_cradle", color=(0.72, 0.70, 0.66, 1.0))
    bearing_black = model.material("black_bearing_pads", color=(0.015, 0.015, 0.017, 1.0))

    axis_height = 0.105

    base = model.part("base")
    base.visual(
        Box((0.40, 0.28, 0.026)),
        origin=Origin(xyz=(0.0, 0.0, 0.013)),
        material=dark,
        name="base_plate",
    )
    # Low bridge-style bearing supports leave a visible window around the roll pins.
    for x in (-0.166, 0.166):
        base.visual(
            Box((0.046, 0.108, 0.056)),
            origin=Origin(xyz=(x, 0.0, 0.054)),
            material=dark,
            name=f"roll_saddle_{x:+.3f}",
        )
        for y in (-0.043, 0.043):
            base.visual(
                Box((0.046, 0.020, 0.104)),
                origin=Origin(xyz=(x, y, 0.078)),
                material=dark,
                name=f"roll_cheek_{x:+.3f}_{y:+.3f}",
            )
        base.visual(
            Box((0.046, 0.108, 0.014)),
            origin=Origin(xyz=(x, 0.0, 0.1245)),
            material=dark,
            name=f"roll_cap_{x:+.3f}",
        )
        base.visual(
            Cylinder(radius=0.020, length=0.006),
            origin=Origin(xyz=(x, 0.0, axis_height), rpy=(0.0, pi / 2.0, 0.0)),
            material=bearing_black,
            name=f"roll_bearing_face_{x:+.3f}",
        )

    outer = model.part("outer_frame")
    # A compact rectangular member, not a closed enclosure, surrounds the inner cradle.
    outer.visual(
        Box((0.260, 0.024, 0.020)),
        origin=Origin(xyz=(0.0, 0.078, 0.0)),
        material=outer_metal,
        name="outer_side_0",
    )
    outer.visual(
        Box((0.260, 0.024, 0.020)),
        origin=Origin(xyz=(0.0, -0.078, 0.0)),
        material=outer_metal,
        name="outer_side_1",
    )
    outer.visual(
        Box((0.024, 0.180, 0.020)),
        origin=Origin(xyz=(0.118, 0.0, 0.0)),
        material=outer_metal,
        name="outer_end_0",
    )
    outer.visual(
        Box((0.024, 0.180, 0.020)),
        origin=Origin(xyz=(-0.118, 0.0, 0.0)),
        material=outer_metal,
        name="outer_end_1",
    )
    for x in (-0.146, 0.146):
        outer.visual(
            Cylinder(radius=0.013, length=0.040),
            origin=Origin(xyz=(x, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
            material=outer_metal,
            name=f"roll_pin_{x:+.3f}",
        )
        outer.visual(
            Cylinder(radius=0.017, length=0.006),
            origin=Origin(xyz=(x * 0.89, 0.0, 0.0), rpy=(0.0, pi / 2.0, 0.0)),
            material=bearing_black,
            name=f"roll_washer_{x:+.3f}",
        )

    inner = model.part("inner_cradle")
    inner.visual(
        Cylinder(radius=0.010, length=0.132),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(-pi / 2.0, 0.0, 0.0)),
        material=inner_metal,
        name="pitch_shaft",
    )
    inner.visual(
        Box((0.122, 0.064, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, -0.026)),
        material=inner_metal,
        name="cradle_floor",
    )
    inner.visual(
        Box((0.020, 0.064, 0.038)),
        origin=Origin(xyz=(0.0, 0.0, -0.012)),
        material=inner_metal,
        name="cradle_spine",
    )
    for x in (-0.050, 0.050):
        inner.visual(
            Box((0.014, 0.064, 0.044)),
            origin=Origin(xyz=(x, 0.0, -0.012)),
            material=inner_metal,
            name=f"cradle_rib_{x:+.3f}",
        )

    model.articulation(
        "base_to_outer_frame",
        ArticulationType.REVOLUTE,
        parent=base,
        child=outer,
        origin=Origin(xyz=(0.0, 0.0, axis_height)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=8.0, velocity=2.0, lower=-0.55, upper=0.55),
    )
    model.articulation(
        "outer_frame_to_inner_cradle",
        ArticulationType.REVOLUTE,
        parent=outer,
        child=inner,
        origin=Origin(),
        axis=(0.0, 1.0, 0.0),
        motion_limits=MotionLimits(effort=4.0, velocity=2.5, lower=-0.65, upper=0.65),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    outer = object_model.get_part("outer_frame")
    inner = object_model.get_part("inner_cradle")
    roll = object_model.get_articulation("base_to_outer_frame")
    pitch = object_model.get_articulation("outer_frame_to_inner_cradle")

    ctx.check(
        "roll and pitch axes are perpendicular",
        abs(sum(a * b for a, b in zip(roll.axis, pitch.axis))) < 1.0e-6,
        details=f"roll_axis={roll.axis}, pitch_axis={pitch.axis}",
    )
    ctx.expect_within(
        inner,
        outer,
        axes="xy",
        margin=0.002,
        name="inner cradle nests inside outer frame footprint",
    )
    ctx.expect_gap(
        outer,
        base,
        axis="z",
        negative_elem="base_plate",
        min_gap=0.012,
        name="outer frame rides above the low base",
    )

    with ctx.pose({roll: 0.55}):
        ctx.expect_gap(
            outer,
            base,
            axis="z",
            negative_elem="base_plate",
            min_gap=0.010,
            name="rolled outer frame clears the base",
        )
    with ctx.pose({pitch: 0.65}):
        ctx.expect_gap(
            inner,
            base,
            axis="z",
            negative_elem="base_plate",
            min_gap=0.006,
            name="pitched inner cradle clears the base",
        )

    return ctx.report()


object_model = build_object_model()
