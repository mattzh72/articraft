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
    model = ArticulatedObject(name="wall_backed_telescoping_slide")

    galvanized = model.material("galvanized_steel", rgba=(0.62, 0.66, 0.66, 1.0))
    dark_steel = model.material("dark_steel", rgba=(0.12, 0.13, 0.13, 1.0))
    wear_polymer = model.material("black_wear_pads", rgba=(0.015, 0.015, 0.012, 1.0))
    wall_paint = model.material("painted_wall_plate", rgba=(0.74, 0.75, 0.72, 1.0))

    fixed = model.part("back_support")
    fixed.visual(
        Box((0.030, 0.280, 0.200)),
        origin=Origin(xyz=(0.0, 0.0, 0.100)),
        material=wall_paint,
        name="wall_plate",
    )

    # Outer U-channel, welded directly to the wall plate.  It is open on top so
    # the nested section visibly rides in a real guide instead of a solid block.
    fixed.visual(
        Box((0.500, 0.100, 0.016)),
        origin=Origin(xyz=(0.265, 0.0, 0.064)),
        material=galvanized,
        name="outer_bottom",
    )
    fixed.visual(
        Box((0.500, 0.012, 0.064)),
        origin=Origin(xyz=(0.265, 0.044, 0.088)),
        material=galvanized,
        name="outer_side_0",
    )
    fixed.visual(
        Box((0.500, 0.012, 0.064)),
        origin=Origin(xyz=(0.265, -0.044, 0.088)),
        material=galvanized,
        name="outer_side_1",
    )
    fixed.visual(
        Box((0.026, 0.124, 0.090)),
        origin=Origin(xyz=(0.026, 0.0, 0.088)),
        material=galvanized,
        name="welded_collar",
    )

    # Wall fasteners make the rigid backing read as a bolted support plate.
    for i, (y, z) in enumerate(
        ((-0.105, 0.044), (0.105, 0.044), (-0.105, 0.156), (0.105, 0.156))
    ):
        fixed.visual(
            Cylinder(radius=0.014, length=0.006),
            origin=Origin(xyz=(0.018, y, z), rpy=(0.0, math.pi / 2.0, 0.0)),
            material=dark_steel,
            name=f"screw_head_{i}",
        )
        fixed.visual(
            Box((0.0015, 0.020, 0.003)),
            origin=Origin(xyz=(0.0215, y, z)),
            material=wall_paint,
            name=f"screw_slot_{i}",
        )

    moving = model.part("moving_section")
    moving.visual(
        Box((0.440, 0.052, 0.032)),
        # With the child frame at the mouth of the fixed guide, this member
        # extends backward inside the channel for retained insertion and
        # projects slightly from the guide even at the collapsed pose.
        origin=Origin(xyz=(-0.120, 0.0, 0.0)),
        material=galvanized,
        name="inner_bar",
    )
    moving.visual(
        Box((0.360, 0.016, 0.008)),
        origin=Origin(xyz=(-0.140, 0.018, -0.020)),
        material=wear_polymer,
        name="bottom_pad_0",
    )
    moving.visual(
        Box((0.360, 0.016, 0.008)),
        origin=Origin(xyz=(-0.140, -0.018, -0.020)),
        material=wear_polymer,
        name="bottom_pad_1",
    )
    moving.visual(
        Box((0.360, 0.012, 0.020)),
        origin=Origin(xyz=(-0.140, 0.032, 0.0)),
        material=wear_polymer,
        name="side_pad_0",
    )
    moving.visual(
        Box((0.360, 0.012, 0.020)),
        origin=Origin(xyz=(-0.140, -0.032, 0.0)),
        material=wear_polymer,
        name="side_pad_1",
    )
    moving.visual(
        Box((0.018, 0.072, 0.052)),
        origin=Origin(xyz=(0.109, 0.0, 0.0)),
        material=dark_steel,
        name="end_stop",
    )

    model.articulation(
        "guide_slide",
        ArticulationType.PRISMATIC,
        parent=fixed,
        child=moving,
        # The joint frame is the front mouth of the fixed guide. Positive
        # travel projects the nested section outward from the wall.
        origin=Origin(xyz=(0.515, 0.0, 0.096)),
        axis=(1.0, 0.0, 0.0),
        motion_limits=MotionLimits(effort=85.0, velocity=0.35, lower=0.0, upper=0.220),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    fixed = object_model.get_part("back_support")
    moving = object_model.get_part("moving_section")
    slide = object_model.get_articulation("guide_slide")

    ctx.check(
        "slide is prismatic",
        slide.articulation_type == ArticulationType.PRISMATIC,
        details=f"type={slide.articulation_type}",
    )

    with ctx.pose({slide: 0.0}):
        ctx.expect_overlap(
            moving,
            fixed,
            axes="x",
            elem_a="inner_bar",
            elem_b="outer_bottom",
            min_overlap=0.30,
            name="collapsed section remains deeply inserted",
        )
        ctx.expect_gap(
            moving,
            fixed,
            axis="z",
            positive_elem="bottom_pad_0",
            negative_elem="outer_bottom",
            max_gap=0.001,
            max_penetration=0.0005,
            name="lower wear pad rests on guide floor",
        )

    rest_pos = ctx.part_world_position(moving)
    with ctx.pose({slide: 0.220}):
        ctx.expect_overlap(
            moving,
            fixed,
            axes="x",
            elem_a="inner_bar",
            elem_b="outer_bottom",
            min_overlap=0.10,
            name="extended section retains guide engagement",
        )
        ctx.expect_gap(
            fixed,
            moving,
            axis="y",
            positive_elem="outer_side_0",
            negative_elem="side_pad_0",
            max_gap=0.001,
            max_penetration=0.0005,
            name="positive side shoe runs on guide wall",
        )
        ctx.expect_gap(
            moving,
            fixed,
            axis="y",
            positive_elem="side_pad_1",
            negative_elem="outer_side_1",
            max_gap=0.001,
            max_penetration=0.0005,
            name="negative side shoe runs on guide wall",
        )
        extended_pos = ctx.part_world_position(moving)

    ctx.check(
        "positive joint travel projects outward",
        rest_pos is not None
        and extended_pos is not None
        and extended_pos[0] > rest_pos[0] + 0.200,
        details=f"rest={rest_pos}, extended={extended_pos}",
    )

    return ctx.report()


object_model = build_object_model()
