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
    model = ArticulatedObject(name="box_frame_twin_column_lift")

    painted_steel = Material("painted_steel", color=(0.08, 0.10, 0.13, 1.0))
    bridge_paint = Material("bridge_paint", color=(0.11, 0.14, 0.18, 1.0))
    polished_steel = Material("polished_steel", color=(0.72, 0.75, 0.76, 1.0))
    carriage_paint = Material("carriage_paint", color=(0.95, 0.42, 0.10, 1.0))
    bronze_liner = Material("bronze_liner", color=(0.65, 0.45, 0.18, 1.0))
    dark_fastener = Material("dark_fastener", color=(0.02, 0.02, 0.025, 1.0))

    frame = model.part("frame")
    frame.visual(
        Box((0.95, 0.48, 0.10)),
        origin=Origin(xyz=(0.0, 0.0, 0.05)),
        material=painted_steel,
        name="lower_base",
    )
    frame.visual(
        Box((0.82, 0.24, 0.10)),
        origin=Origin(xyz=(0.0, 0.0, 1.20)),
        material=bridge_paint,
        name="upper_bridge",
    )

    column_x = 0.28
    for index, x in enumerate((-column_x, column_x)):
        frame.visual(
            Cylinder(radius=0.035, length=1.05),
            origin=Origin(xyz=(x, 0.0, 0.625)),
            material=polished_steel,
            name=f"guide_column_{index}",
        )
        frame.visual(
            Box((0.16, 0.18, 0.06)),
            origin=Origin(xyz=(x, 0.0, 0.13)),
            material=bridge_paint,
            name=f"lower_column_block_{index}",
        )
        frame.visual(
            Box((0.16, 0.18, 0.06)),
            origin=Origin(xyz=(x, 0.0, 1.12)),
            material=bridge_paint,
            name=f"upper_column_block_{index}",
        )
        for sx in (-1.0, 1.0):
            for sy in (-1.0, 1.0):
                frame.visual(
                    Cylinder(radius=0.012, length=0.008),
                    origin=Origin(xyz=(x + sx * 0.052, sy * 0.060, 0.164)),
                    material=dark_fastener,
                    name=f"base_bolt_{index}_{sx}_{sy}",
                )

    carriage = model.part("carriage")

    # The child frame sits at the center of the moving crosshead.  The twin
    # guide sleeves wrap around the fixed columns with visible clearance, so
    # the moving part reads as constrained without relying on collision.
    carriage.visual(
        Box((0.392, 0.045, 0.18)),
        origin=Origin(xyz=(0.0, -0.107, 0.0)),
        material=carriage_paint,
        name="front_crosshead",
    )
    carriage.visual(
        Box((0.32, 0.052, 0.26)),
        origin=Origin(xyz=(0.0, -0.155, 0.0)),
        material=carriage_paint,
        name="tool_plate",
    )
    carriage.visual(
        Cylinder(radius=0.065, length=0.036),
        origin=Origin(xyz=(0.0, -0.197, 0.0), rpy=(pi / 2.0, 0.0, 0.0)),
        material=bridge_paint,
        name="mounting_boss",
    )
    for index, x in enumerate((-0.035, 0.035)):
        carriage.visual(
            Cylinder(radius=0.016, length=0.010),
            origin=Origin(xyz=(x, -0.219, 0.045), rpy=(pi / 2.0, 0.0, 0.0)),
            material=dark_fastener,
            name=f"plate_bolt_top_{index}",
        )
        carriage.visual(
            Cylinder(radius=0.016, length=0.010),
            origin=Origin(xyz=(x, -0.219, -0.045), rpy=(pi / 2.0, 0.0, 0.0)),
            material=dark_fastener,
            name=f"plate_bolt_bottom_{index}",
        )

    sleeve_outer = 0.17
    sleeve_hole = 0.104
    sleeve_wall = (sleeve_outer - sleeve_hole) / 2.0
    sleeve_height = 0.24
    wall_offset = sleeve_hole / 2.0 + sleeve_wall / 2.0
    for index, x in enumerate((-column_x, column_x)):
        carriage.visual(
            Box((sleeve_wall, sleeve_outer, sleeve_height)),
            origin=Origin(xyz=(x - wall_offset, 0.0, 0.0)),
            material=carriage_paint,
            name=f"sleeve_{index}_outer_wall",
        )
        carriage.visual(
            Box((sleeve_wall, sleeve_outer, sleeve_height)),
            origin=Origin(xyz=(x + wall_offset, 0.0, 0.0)),
            material=carriage_paint,
            name=f"sleeve_{index}_inner_wall",
        )
        carriage.visual(
            Box((sleeve_outer, sleeve_wall, sleeve_height)),
            origin=Origin(xyz=(x, -wall_offset, 0.0)),
            material=carriage_paint,
            name=f"sleeve_{index}_front_wall",
        )
        carriage.visual(
            Box((sleeve_outer, sleeve_wall, sleeve_height)),
            origin=Origin(xyz=(x, wall_offset, 0.0)),
            material=carriage_paint,
            name=f"sleeve_{index}_rear_wall",
        )
        carriage.visual(
            Box((0.012, 0.022, sleeve_height - 0.020)),
            origin=Origin(xyz=(x, -0.043, 0.0)),
            material=bronze_liner,
            name=f"sleeve_{index}_front_liner",
        )
        carriage.visual(
            Box((0.012, 0.022, sleeve_height - 0.020)),
            origin=Origin(xyz=(x, 0.043, 0.0)),
            material=bronze_liner,
            name=f"sleeve_{index}_rear_liner",
        )

    model.articulation(
        "frame_to_carriage",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=carriage,
        origin=Origin(xyz=(0.0, 0.0, 0.34)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=500.0, velocity=0.18, lower=0.0, upper=0.55),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    carriage = object_model.get_part("carriage")
    slide = object_model.get_articulation("frame_to_carriage")

    for index in (0, 1):
        ctx.allow_overlap(
            frame,
            carriage,
            elem_a=f"guide_column_{index}",
            elem_b=f"sleeve_{index}_front_liner",
            reason="The bronze guide liner is modeled with a tiny preload into the polished column so the sliding carriage has a physical guide contact.",
        )
        ctx.allow_overlap(
            carriage,
            frame,
            elem_a=f"sleeve_{index}_rear_liner",
            elem_b=f"guide_column_{index}",
            reason="The rear bronze liner has the same small linear-guide preload against the column.",
        )
        ctx.expect_gap(
            frame,
            carriage,
            axis="y",
            positive_elem=f"guide_column_{index}",
            negative_elem=f"sleeve_{index}_front_liner",
            max_penetration=0.004,
            name=f"front guide liner preload is local on column {index}",
        )
        ctx.expect_gap(
            carriage,
            frame,
            axis="y",
            positive_elem=f"sleeve_{index}_rear_liner",
            negative_elem=f"guide_column_{index}",
            max_penetration=0.004,
            name=f"rear guide liner preload is local on column {index}",
        )

    ctx.expect_overlap(
        carriage,
        frame,
        axes="z",
        elem_a="sleeve_0_front_wall",
        elem_b="guide_column_0",
        min_overlap=0.20,
        name="carriage sleeve overlaps first guide column in height",
    )
    ctx.expect_overlap(
        carriage,
        frame,
        axes="z",
        elem_a="sleeve_1_front_wall",
        elem_b="guide_column_1",
        min_overlap=0.20,
        name="carriage sleeve overlaps second guide column in height",
    )
    ctx.expect_gap(
        frame,
        carriage,
        axis="y",
        positive_elem="guide_column_0",
        negative_elem="sleeve_0_front_wall",
        min_gap=0.010,
        name="first front sleeve wall clears column",
    )
    ctx.expect_gap(
        carriage,
        frame,
        axis="y",
        positive_elem="sleeve_1_rear_wall",
        negative_elem="guide_column_1",
        min_gap=0.010,
        name="second rear sleeve wall clears column",
    )

    rest_pos = ctx.part_world_position(carriage)
    with ctx.pose({slide: 0.55}):
        ctx.expect_overlap(
            carriage,
            frame,
            axes="z",
            elem_a="sleeve_0_front_wall",
            elem_b="guide_column_0",
            min_overlap=0.20,
            name="raised carriage remains engaged on first column",
        )
        ctx.expect_overlap(
            carriage,
            frame,
            axes="z",
            elem_a="sleeve_1_front_wall",
            elem_b="guide_column_1",
            min_overlap=0.20,
            name="raised carriage remains engaged on second column",
        )
        raised_pos = ctx.part_world_position(carriage)

    ctx.check(
        "prismatic joint lifts carriage along columns",
        rest_pos is not None
        and raised_pos is not None
        and raised_pos[2] > rest_pos[2] + 0.50,
        details=f"rest={rest_pos}, raised={raised_pos}",
    )

    return ctx.report()


object_model = build_object_model()
