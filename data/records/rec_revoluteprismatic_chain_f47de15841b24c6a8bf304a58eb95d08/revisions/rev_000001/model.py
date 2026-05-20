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
    model = ArticulatedObject(name="low_profile_pivot_arm")

    base_finish = model.material("charcoal_powder_coat", color=(0.06, 0.065, 0.07, 1.0))
    rubber = model.material("black_rubber", color=(0.012, 0.012, 0.012, 1.0))
    steel = model.material("brushed_steel", color=(0.58, 0.60, 0.60, 1.0))
    arm_finish = model.material("warm_gray_arm", color=(0.34, 0.36, 0.37, 1.0))
    rail_finish = model.material("dark_guide_rails", color=(0.18, 0.19, 0.20, 1.0))
    slider_finish = model.material("satin_slider", color=(0.78, 0.80, 0.78, 1.0))
    nose_finish = model.material("orange_nose_cap", color=(0.96, 0.38, 0.08, 1.0))

    # A wide, heavy base keeps the low arm visibly grounded while leaving the
    # pivot axis near the rear of the top plate, like a compact positioning arm.
    base = model.part("base")
    base.visual(
        Box((0.62, 0.38, 0.040)),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material=base_finish,
        name="ground_plate",
    )
    for i, (x, y) in enumerate(
        ((-0.235, -0.145), (-0.235, 0.145), (0.235, -0.145), (0.235, 0.145))
    ):
        base.visual(
            Box((0.105, 0.058, 0.010)),
            origin=Origin(xyz=(x, y, 0.005)),
            material=rubber,
            name=f"foot_{i}",
        )
    base.visual(
        Cylinder(radius=0.086, length=0.060),
        origin=Origin(xyz=(-0.180, 0.0, 0.075)),
        material=steel,
        name="pivot_bearing",
    )
    base.visual(
        Cylinder(radius=0.110, length=0.010),
        origin=Origin(xyz=(-0.180, 0.0, 0.050)),
        material=steel,
        name="bearing_flange",
    )
    for i, (dx, dy) in enumerate(((0.055, 0.050), (-0.055, 0.050), (0.055, -0.050), (-0.055, -0.050))):
        base.visual(
            Cylinder(radius=0.012, length=0.008),
            origin=Origin(xyz=(-0.180 + dx, dy, 0.049)),
            material=steel,
            name=f"bolt_{i}",
        )

    # The arm part frame is the vertical pivot axis at the top of the bearing.
    # Geometry extends along local +X, so yawing the part swings the shallow
    # guide channel across the broad base.
    arm = model.part("arm")
    arm.visual(
        Cylinder(radius=0.083, length=0.050),
        origin=Origin(xyz=(0.0, 0.0, 0.025)),
        material=arm_finish,
        name="pivot_hub",
    )
    arm.visual(
        Box((0.540, 0.150, 0.016)),
        origin=Origin(xyz=(0.350, 0.0, 0.008)),
        material=arm_finish,
        name="channel_floor",
    )
    arm.visual(
        Box((0.530, 0.028, 0.047)),
        origin=Origin(xyz=(0.355, 0.064, 0.0385)),
        material=rail_finish,
        name="rail_0",
    )
    arm.visual(
        Box((0.530, 0.028, 0.047)),
        origin=Origin(xyz=(0.355, -0.064, 0.0385)),
        material=rail_finish,
        name="rail_1",
    )
    arm.visual(
        Box((0.150, 0.150, 0.015)),
        origin=Origin(xyz=(0.145, 0.0, 0.0535)),
        material=arm_finish,
        name="rear_keeper",
    )

    # The child frame is the mouth of the arm guide.  The hidden tongue extends
    # backward into the channel at q=0 and remains well retained at full travel.
    slider = model.part("slider")
    slider.visual(
        Box((0.420, 0.070, 0.028)),
        origin=Origin(xyz=(-0.210, 0.0, 0.030)),
        material=slider_finish,
        name="guide_tongue",
    )
    slider.visual(
        Box((0.130, 0.112, 0.042)),
        origin=Origin(xyz=(0.065, 0.0, 0.035)),
        material=nose_finish,
        name="nose_block",
    )
    slider.visual(
        Cylinder(radius=0.025, length=0.112),
        origin=Origin(xyz=(0.130, 0.0, 0.035), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=nose_finish,
        name="rounded_tip",
    )
    slider.visual(
        Box((0.018, 0.084, 0.020)),
        origin=Origin(xyz=(-0.005, 0.0, 0.050)),
        material=slider_finish,
        name="top_stop",
    )

    model.articulation(
        "base_to_arm",
        ArticulationType.REVOLUTE,
        parent=base,
        child=arm,
        origin=Origin(xyz=(-0.180, 0.0, 0.105)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=18.0, velocity=1.2, lower=-0.85, upper=0.85),
    )
    model.articulation(
        "arm_to_slider",
        ArticulationType.PRISMATIC,
        parent=arm,
        child=slider,
        origin=Origin(xyz=(0.620, 0.0, 0.0)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=45.0, velocity=0.35, lower=0.0, upper=0.180),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    arm = object_model.get_part("arm")
    slider = object_model.get_part("slider")
    arm_joint = object_model.get_articulation("base_to_arm")
    slide_joint = object_model.get_articulation("arm_to_slider")

    ctx.expect_gap(
        arm,
        base,
        axis="z",
        positive_elem="pivot_hub",
        negative_elem="pivot_bearing",
        max_gap=0.001,
        max_penetration=0.0,
        name="arm hub sits on pivot bearing",
    )
    ctx.expect_overlap(
        arm,
        base,
        axes="xy",
        elem_a="pivot_hub",
        elem_b="pivot_bearing",
        min_overlap=0.14,
        name="pivot hub is centered on bearing",
    )
    ctx.expect_within(
        slider,
        arm,
        axes="yz",
        inner_elem="guide_tongue",
        margin=0.001,
        name="slider tongue rides inside arm channel",
    )
    ctx.expect_gap(
        slider,
        arm,
        axis="z",
        positive_elem="guide_tongue",
        negative_elem="channel_floor",
        max_gap=0.001,
        max_penetration=0.0,
        name="slider tongue is supported by floor",
    )
    ctx.expect_overlap(
        slider,
        arm,
        axes="x",
        elem_a="guide_tongue",
        elem_b="channel_floor",
        min_overlap=0.35,
        name="retracted slider remains deeply inserted",
    )

    rest_pos = ctx.part_world_position(slider)
    with ctx.pose({slide_joint: 0.180}):
        ctx.expect_within(
            slider,
            arm,
            axes="yz",
            inner_elem="guide_tongue",
            margin=0.001,
            name="extended tongue stays captured in channel",
        )
        ctx.expect_overlap(
            slider,
            arm,
            axes="x",
            elem_a="guide_tongue",
            elem_b="channel_floor",
            min_overlap=0.20,
            name="extended slider retains insertion",
        )
        extended_pos = ctx.part_world_position(slider)

    ctx.check(
        "prismatic joint extends nose forward",
        rest_pos is not None and extended_pos is not None and extended_pos[0] > rest_pos[0] + 0.16,
        details=f"rest={rest_pos}, extended={extended_pos}",
    )

    with ctx.pose({arm_joint: 0.55}):
        swung_pos = ctx.part_world_position(slider)
    ctx.check(
        "revolute joint swings arm about base",
        rest_pos is not None and swung_pos is not None and abs(swung_pos[1]) > 0.25,
        details=f"rest={rest_pos}, swung={swung_pos}",
    )

    return ctx.report()


object_model = build_object_model()
