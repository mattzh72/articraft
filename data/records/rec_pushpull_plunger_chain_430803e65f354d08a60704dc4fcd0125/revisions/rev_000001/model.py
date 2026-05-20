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
    model = ArticulatedObject(name="push_pull_plunger_chain")

    dark_anodized = Material("dark_anodized", rgba=(0.08, 0.085, 0.09, 1.0))
    brushed_steel = Material("brushed_steel", rgba=(0.72, 0.70, 0.66, 1.0))
    black_rubber = Material("black_rubber", rgba=(0.015, 0.015, 0.014, 1.0))
    lever_red = Material("red_lever", rgba=(0.65, 0.05, 0.035, 1.0))
    brass = Material("brass_pin", rgba=(0.90, 0.63, 0.20, 1.0))

    housing = model.part("housing")
    # Four overlapping rails make a visibly hollow rectangular guide around the
    # plunger axis, with an open bore rather than a solid placeholder block.
    housing.visual(
        Box((0.220, 0.070, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, 0.029)),
        material=dark_anodized,
        name="top_rail",
    )
    housing.visual(
        Box((0.220, 0.070, 0.012)),
        origin=Origin(xyz=(0.0, 0.0, -0.029)),
        material=dark_anodized,
        name="bottom_rail",
    )
    housing.visual(
        Box((0.220, 0.012, 0.060)),
        origin=Origin(xyz=(0.0, 0.029, 0.0)),
        material=dark_anodized,
        name="side_rail_0",
    )
    housing.visual(
        Box((0.220, 0.012, 0.060)),
        origin=Origin(xyz=(0.0, -0.029, 0.0)),
        material=dark_anodized,
        name="side_rail_1",
    )
    housing.visual(
        Box((0.024, 0.014, 0.014)),
        origin=Origin(xyz=(-0.020, 0.0, 0.016)),
        material=brass,
        name="upper_bearing",
    )
    housing.visual(
        Box((0.024, 0.014, 0.014)),
        origin=Origin(xyz=(-0.020, 0.0, -0.016)),
        material=brass,
        name="lower_bearing",
    )
    # A front clevis rises from the guide and carries the transverse lever pin.
    housing.visual(
        Box((0.028, 0.014, 0.056)),
        origin=Origin(xyz=(0.115, 0.031, 0.039)),
        material=dark_anodized,
        name="clevis_0",
    )
    housing.visual(
        Box((0.028, 0.014, 0.056)),
        origin=Origin(xyz=(0.115, -0.031, 0.039)),
        material=dark_anodized,
        name="clevis_1",
    )
    housing.visual(
        Cylinder(radius=0.0055, length=0.086),
        origin=Origin(xyz=(0.115, 0.0, 0.055), rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=brass,
        name="pivot_pin",
    )

    plunger = model.part("plunger")
    plunger.visual(
        Cylinder(radius=0.009, length=0.220),
        origin=Origin(xyz=(-0.005, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brushed_steel,
        name="shaft",
    )
    plunger.visual(
        Cylinder(radius=0.012, length=0.012),
        origin=Origin(xyz=(0.101, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=brushed_steel,
        name="tip",
    )
    plunger.visual(
        Cylinder(radius=0.022, length=0.024),
        origin=Origin(xyz=(-0.124, 0.0, 0.0), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black_rubber,
        name="pull_knob",
    )

    lever = model.part("lever")
    lever.visual(
        Cylinder(radius=0.013, length=0.022),
        origin=Origin(rpy=(-math.pi / 2.0, 0.0, 0.0)),
        material=lever_red,
        name="hub",
    )
    lever.visual(
        Box((0.012, 0.018, 0.042)),
        origin=Origin(xyz=(0.0, 0.0, -0.034)),
        material=lever_red,
        name="arm",
    )
    lever.visual(
        Box((0.012, 0.026, 0.016)),
        origin=Origin(xyz=(0.0, 0.0, -0.055)),
        material=lever_red,
        name="drive_pad",
    )

    model.articulation(
        "housing_to_plunger",
        ArticulationType.PRISMATIC,
        parent=housing,
        child=plunger,
        origin=Origin(),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=35.0, velocity=0.25, lower=0.0, upper=0.050),
    )
    model.articulation(
        "housing_to_lever",
        ArticulationType.REVOLUTE,
        parent=housing,
        child=lever,
        origin=Origin(xyz=(0.115, 0.0, 0.055)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(
            effort=4.0,
            velocity=2.0,
            lower=0.0,
            upper=math.radians(45.0),
        ),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    housing = object_model.get_part("housing")
    plunger = object_model.get_part("plunger")
    lever = object_model.get_part("lever")
    slide = object_model.get_articulation("housing_to_plunger")
    hinge = object_model.get_articulation("housing_to_lever")

    ctx.allow_overlap(
        housing,
        lever,
        elem_a="pivot_pin",
        elem_b="hub",
        reason="The fixed transverse pin is intentionally captured inside the lever hub.",
    )
    ctx.allow_overlap(
        plunger,
        lever,
        elem_a="tip",
        elem_b="drive_pad",
        reason="At the fully pushed pose the metal tip is modeled with slight local compression against the lever drive pad.",
    )
    ctx.expect_overlap(
        housing,
        lever,
        axes="yz",
        elem_a="pivot_pin",
        elem_b="hub",
        min_overlap=0.010,
        name="pin passes through lever hub",
    )
    ctx.expect_within(
        plunger,
        housing,
        axes="yz",
        inner_elem="shaft",
        margin=0.0,
        name="plunger shaft remains centered in guide envelope",
    )
    ctx.expect_gap(
        housing,
        plunger,
        axis="z",
        positive_elem="top_rail",
        negative_elem="shaft",
        min_gap=0.010,
        name="shaft clears upper guide rail",
    )
    ctx.expect_gap(
        plunger,
        housing,
        axis="z",
        positive_elem="shaft",
        negative_elem="bottom_rail",
        min_gap=0.010,
        name="shaft clears lower guide rail",
    )

    ctx.check(
        "plunger travel is 50 mm",
        slide.motion_limits is not None
        and abs((slide.motion_limits.upper or 0.0) - 0.050) < 1e-6
        and abs(slide.motion_limits.lower or 0.0) < 1e-6,
        details=f"limits={slide.motion_limits}",
    )
    ctx.check(
        "lever travel is about 45 degrees",
        hinge.motion_limits is not None
        and abs((hinge.motion_limits.upper or 0.0) - math.radians(45.0)) < 1e-6,
        details=f"limits={hinge.motion_limits}",
    )

    rest_plunger = ctx.part_world_position(plunger)
    rest_pad = ctx.part_element_world_aabb(lever, elem="drive_pad")
    with ctx.pose({slide: 0.050, hinge: math.radians(45.0)}):
        extended_plunger = ctx.part_world_position(plunger)
        extended_pad = ctx.part_element_world_aabb(lever, elem="drive_pad")
        ctx.expect_gap(
            lever,
            plunger,
            axis="x",
            positive_elem="drive_pad",
            negative_elem="tip",
            max_gap=0.006,
            max_penetration=0.015,
            name="pushed tip bears on lever pad",
        )

    rest_pad_center_x = None if rest_pad is None else (rest_pad[0][0] + rest_pad[1][0]) / 2.0
    extended_pad_center_x = (
        None if extended_pad is None else (extended_pad[0][0] + extended_pad[1][0]) / 2.0
    )
    ctx.check(
        "plunger extends forward",
        rest_plunger is not None
        and extended_plunger is not None
        and extended_plunger[0] > rest_plunger[0] + 0.045,
        details=f"rest={rest_plunger}, extended={extended_plunger}",
    )
    ctx.check(
        "front lever swings forward",
        rest_pad_center_x is not None
        and extended_pad_center_x is not None
        and extended_pad_center_x > rest_pad_center_x + 0.030,
        details=f"rest_x={rest_pad_center_x}, extended_x={extended_pad_center_x}",
    )

    return ctx.report()


object_model = build_object_model()
