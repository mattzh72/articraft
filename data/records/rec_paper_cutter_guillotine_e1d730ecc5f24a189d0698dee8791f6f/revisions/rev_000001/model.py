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
    model = ArticulatedObject(name="heavy_office_paper_cutter")

    dark_steel = model.material("dark_steel", rgba=(0.10, 0.11, 0.11, 1.0))
    brushed_steel = model.material("brushed_steel", rgba=(0.62, 0.64, 0.62, 1.0))
    blade_steel = model.material("blade_steel", rgba=(0.82, 0.84, 0.80, 1.0))
    black_plastic = model.material("black_plastic", rgba=(0.015, 0.015, 0.014, 1.0))
    bed_green = model.material("printed_green_bed", rgba=(0.58, 0.68, 0.56, 1.0))
    print_gray = model.material("printed_gray_lines", rgba=(0.08, 0.10, 0.09, 1.0))
    ruler_white = model.material("ruler_white", rgba=(0.93, 0.91, 0.82, 1.0))

    base = model.part("base")

    # A heavy, real-world desktop guillotine base, 72 cm long by 38 cm wide.
    base.visual(
        Box((0.72, 0.38, 0.045)),
        origin=Origin(xyz=(0.0, 0.0, 0.0225)),
        material=dark_steel,
        name="base_plate",
    )
    base.visual(
        Box((0.58, 0.30, 0.003)),
        origin=Origin(xyz=(0.05, 0.0, 0.0460)),
        material=bed_green,
        name="alignment_bed",
    )

    # Raised, fixed side fence/ruler along one long edge of the bed.
    base.visual(
        Box((0.64, 0.028, 0.040)),
        origin=Origin(xyz=(0.02, 0.172, 0.065)),
        material=brushed_steel,
        name="side_fence",
    )
    base.visual(
        Box((0.61, 0.004, 0.006)),
        origin=Origin(xyz=(0.03, 0.156, 0.084)),
        material=ruler_white,
        name="fence_ruler_strip",
    )

    # Printed alignment bed: square grid, long rule lines, and diagonal angle guides.
    for i, x in enumerate([-0.20, -0.15, -0.10, -0.05, 0.0, 0.05, 0.10, 0.15, 0.20, 0.25, 0.30]):
        thickness = 0.0028 if i % 2 == 0 else 0.0017
        base.visual(
            Box((thickness, 0.285, 0.0008)),
            origin=Origin(xyz=(x, 0.0, 0.0478)),
            material=print_gray,
            name=f"grid_x_{i}",
        )
    for i, y in enumerate([-0.125, -0.10, -0.075, -0.05, -0.025, 0.0, 0.025, 0.05, 0.075, 0.10, 0.125]):
        thickness = 0.0028 if i % 2 == 0 else 0.0017
        base.visual(
            Box((0.56, thickness, 0.0008)),
            origin=Origin(xyz=(0.05, y, 0.0479)),
            material=print_gray,
            name=f"grid_y_{i}",
        )
    for name, yaw, length, x, y in (
        ("guide_45", math.radians(45.0), 0.36, -0.09, -0.030),
        ("guide_60", math.radians(30.0), 0.34, -0.08, -0.060),
        ("guide_30", math.radians(-30.0), 0.30, -0.04, 0.075),
    ):
        base.visual(
            Box((length, 0.0032, 0.0009)),
            origin=Origin(xyz=(x, y, 0.0480), rpy=(0.0, 0.0, yaw)),
            material=print_gray,
            name=name,
        )

    # Rubber feet are slightly proud of the underside and embedded into the base.
    for i, (x, y) in enumerate([(-0.31, -0.15), (0.31, -0.15), (-0.31, 0.15), (0.31, 0.15)]):
        base.visual(
            Cylinder(radius=0.032, length=0.012),
            origin=Origin(xyz=(x, y, -0.004)),
            material=black_plastic,
            name=f"foot_{i}",
        )

    pivot_x = -0.31
    pivot_y = -0.145
    pivot_z = 0.115

    # Robust fixed pivot bracket: bolted pad, two upright cheeks, rear bridge,
    # and visible pin caps outside the gap where the blade hub swings.
    base.visual(
        Box((0.118, 0.115, 0.015)),
        origin=Origin(xyz=(pivot_x, pivot_y, 0.052)),
        material=dark_steel,
        name="pivot_pad",
    )
    base.visual(
        Box((0.085, 0.012, 0.130)),
        origin=Origin(xyz=(pivot_x, pivot_y - 0.046, 0.110)),
        material=dark_steel,
        name="rear_cheek",
    )
    base.visual(
        Box((0.085, 0.012, 0.130)),
        origin=Origin(xyz=(pivot_x, pivot_y + 0.046, 0.110)),
        material=dark_steel,
        name="front_cheek",
    )
    base.visual(
        Box((0.018, 0.104, 0.110)),
        origin=Origin(xyz=(pivot_x - 0.065, pivot_y, 0.105)),
        material=dark_steel,
        name="bracket_bridge",
    )
    for i, y in enumerate((pivot_y - 0.052, pivot_y + 0.052)):
        base.visual(
            Cylinder(radius=0.014, length=0.018),
            origin=Origin(xyz=(pivot_x, y, pivot_z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=brushed_steel,
            name=f"pin_cap_{i}",
        )
    base.visual(
        Cylinder(radius=0.009, length=0.115),
        origin=Origin(xyz=(pivot_x, pivot_y, pivot_z), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=brushed_steel,
        name="pivot_pin",
    )

    blade_arm = model.part("blade_arm")
    blade_arm.visual(
        Cylinder(radius=0.050, length=0.045),
        origin=Origin(xyz=(0.0, 0.0, 0.0), rpy=(math.pi / 2.0, 0.0, 0.0)),
        material=brushed_steel,
        name="hub",
    )
    blade_arm.visual(
        Box((0.590, 0.012, 0.055)),
        origin=Origin(xyz=(0.365, 0.0, -0.039)),
        material=blade_steel,
        name="blade_plate",
    )
    blade_arm.visual(
        Box((0.570, 0.014, 0.006)),
        origin=Origin(xyz=(0.375, 0.0, -0.063)),
        material=blade_steel,
        name="cutting_edge",
    )
    blade_arm.visual(
        Box((0.640, 0.024, 0.022)),
        origin=Origin(xyz=(0.335, 0.0, -0.001)),
        material=dark_steel,
        name="arm_spine",
    )
    for i, x in enumerate((0.445, 0.620)):
        blade_arm.visual(
            Cylinder(radius=0.008, length=0.055),
            origin=Origin(xyz=(x, 0.0, 0.022)),
            material=dark_steel,
            name=f"handle_post_{i}",
        )
    blade_arm.visual(
        Cylinder(radius=0.018, length=0.230),
        origin=Origin(xyz=(0.535, 0.0, 0.055), rpy=(0.0, math.pi / 2.0, 0.0)),
        material=black_plastic,
        name="handle_grip",
    )

    model.articulation(
        "pivot",
        ArticulationType.REVOLUTE,
        parent=base,
        child=blade_arm,
        origin=Origin(xyz=(pivot_x, pivot_y, pivot_z)),
        axis=(0.0, -1.0, 0.0),
        motion_limits=MotionLimits(effort=40.0, velocity=1.4, lower=0.0, upper=math.pi / 2.0),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    base = object_model.get_part("base")
    blade = object_model.get_part("blade_arm")
    pivot = object_model.get_articulation("pivot")

    ctx.allow_overlap(
        base,
        blade,
        elem_a="pivot_pin",
        elem_b="hub",
        reason="The fixed pivot pin intentionally passes through the blade-arm hub as a captured shaft.",
    )

    limits = pivot.motion_limits
    ctx.check(
        "blade arm uses one revolute pivot",
        pivot.articulation_type == ArticulationType.REVOLUTE,
        details=f"type={pivot.articulation_type}",
    )
    ctx.check(
        "pivot travel is about ninety degrees",
        limits is not None
        and limits.lower is not None
        and limits.upper is not None
        and abs(limits.lower) < 1e-6
        and math.radians(85.0) <= limits.upper <= math.radians(95.0),
        details=f"limits={limits}",
    )

    with ctx.pose({pivot: 0.0}):
        ctx.expect_within(
            base,
            blade,
            axes="xz",
            inner_elem="pivot_pin",
            outer_elem="hub",
            margin=0.001,
            name="pivot pin is centered inside the blade hub",
        )
        ctx.expect_overlap(
            base,
            blade,
            axes="y",
            elem_a="pivot_pin",
            elem_b="hub",
            min_overlap=0.035,
            name="pivot pin passes through the hub width",
        )
        ctx.expect_gap(
            blade,
            base,
            axis="z",
            positive_elem="cutting_edge",
            negative_elem="alignment_bed",
            min_gap=0.0005,
            max_gap=0.006,
            name="closed blade rides just above the printed bed",
        )
        ctx.expect_overlap(
            blade,
            base,
            axes="x",
            elem_a="blade_plate",
            elem_b="alignment_bed",
            min_overlap=0.30,
            name="closed blade spans the alignment bed length",
        )

    with ctx.pose({pivot: math.pi / 2.0}):
        ctx.expect_gap(
            blade,
            base,
            axis="z",
            positive_elem="cutting_edge",
            negative_elem="alignment_bed",
            min_gap=0.08,
            name="raised blade clears the bed",
        )

    return ctx.report()


object_model = build_object_model()
