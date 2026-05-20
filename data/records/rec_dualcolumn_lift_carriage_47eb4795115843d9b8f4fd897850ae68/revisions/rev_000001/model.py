from __future__ import annotations

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

import math


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="heavy_lift_carriage_module")

    painted_steel = Material("dark_blue_painted_steel", rgba=(0.05, 0.12, 0.19, 1.0))
    worn_edges = Material("worn_steel_edges", rgba=(0.42, 0.43, 0.40, 1.0))
    black_steel = Material("black_oxide_hardware", rgba=(0.02, 0.02, 0.018, 1.0))
    safety_yellow = Material("safety_yellow_plate", rgba=(0.95, 0.68, 0.05, 1.0))
    bronze = Material("bronze_wear_liners", rgba=(0.72, 0.47, 0.20, 1.0))

    frame = model.part("frame")

    # Welded base frame, built from thick rectangular tube with overlapping
    # corners so the part reads as one welded assembly.
    frame.visual(
        Box((1.34, 0.14, 0.14)),
        origin=Origin(xyz=(0.0, -0.31, 0.07)),
        material=painted_steel,
        name="front_base_rail",
    )
    frame.visual(
        Box((1.34, 0.14, 0.14)),
        origin=Origin(xyz=(0.0, 0.31, 0.07)),
        material=painted_steel,
        name="rear_base_rail",
    )
    frame.visual(
        Box((0.16, 0.76, 0.14)),
        origin=Origin(xyz=(-0.58, 0.0, 0.07)),
        material=painted_steel,
        name="base_side_rail_0",
    )
    frame.visual(
        Box((0.16, 0.76, 0.14)),
        origin=Origin(xyz=(0.58, 0.0, 0.07)),
        material=painted_steel,
        name="base_side_rail_1",
    )
    frame.visual(
        Box((0.86, 0.12, 0.10)),
        origin=Origin(xyz=(0.0, 0.0, 0.16)),
        material=painted_steel,
        name="base_cross_tube",
    )

    # Two rectangular guide columns. They are slightly proud of the base and
    # tucked under the top beam like heavy warehouse lift or press equipment.
    frame.visual(
        Box((0.14, 0.16, 1.88)),
        origin=Origin(xyz=(-0.42, 0.0, 1.06)),
        material=painted_steel,
        name="guide_column_0",
    )
    frame.visual(
        Box((0.14, 0.16, 1.88)),
        origin=Origin(xyz=(0.42, 0.0, 1.06)),
        material=painted_steel,
        name="guide_column_1",
    )
    for index, x in enumerate((-0.42, 0.42)):
        frame.visual(
            Box((0.28, 0.26, 0.035)),
            origin=Origin(xyz=(x, 0.0, 0.157)),
            material=worn_edges,
            name=f"column_foot_plate_{index}",
        )
        frame.visual(
            Box((0.30, 0.045, 0.22)),
            origin=Origin(xyz=(x, -0.11, 0.25)),
            material=painted_steel,
            name=f"front_gusset_{index}",
        )
        frame.visual(
            Box((0.30, 0.045, 0.22)),
            origin=Origin(xyz=(x, 0.11, 0.25)),
            material=painted_steel,
            name=f"rear_gusset_{index}",
        )

    frame.visual(
        Box((1.18, 0.24, 0.18)),
        origin=Origin(xyz=(0.0, 0.0, 2.09)),
        material=painted_steel,
        name="upper_tie_beam",
    )
    frame.visual(
        Box((0.74, 0.08, 0.10)),
        origin=Origin(xyz=(0.0, -0.125, 1.98)),
        material=worn_edges,
        name="upper_front_cap",
    )
    frame.visual(
        Box((0.74, 0.08, 0.10)),
        origin=Origin(xyz=(0.0, 0.125, 1.98)),
        material=worn_edges,
        name="upper_rear_cap",
    )

    carriage = model.part("carriage")
    # The child part frame sits at the center of the moving carriage at the
    # bottom of its travel.  At q=0 the shoe group is mid-low on the columns.
    carriage.visual(
        Box((1.16, 0.10, 0.46)),
        origin=Origin(xyz=(0.0, -0.19, 0.0)),
        material=safety_yellow,
        name="carriage_plate",
    )
    carriage.visual(
        Box((0.94, 0.055, 0.08)),
        origin=Origin(xyz=(0.0, -0.255, 0.17)),
        material=painted_steel,
        name="top_stiffener",
    )
    carriage.visual(
        Box((0.94, 0.055, 0.08)),
        origin=Origin(xyz=(0.0, -0.255, -0.17)),
        material=painted_steel,
        name="bottom_stiffener",
    )
    carriage.visual(
        Box((0.08, 0.06, 0.46)),
        origin=Origin(xyz=(0.0, -0.26, 0.0)),
        material=painted_steel,
        name="center_web",
    )

    shoe_centers = (-0.42, 0.42)
    carriage.visual(
        Box((0.26, 0.06, 0.52)),
        origin=Origin(xyz=(-0.42, -0.125, 0.0)),
        material=safety_yellow,
        name="shoe_0_front",
    )
    carriage.visual(
        Box((0.26, 0.06, 0.52)),
        origin=Origin(xyz=(0.42, -0.125, 0.0)),
        material=safety_yellow,
        name="shoe_1_front",
    )
    for index, x in enumerate(shoe_centers):
        # Four boxed guide-shoe members around each rectangular column. The
        # inner opening is larger than the fixed column by a few millimeters,
        # leaving visible running clearance while still visually capturing the
        # moving plate between the uprights.
        carriage.visual(
            Box((0.26, 0.06, 0.52)),
            origin=Origin(xyz=(x, 0.125, 0.0)),
            material=safety_yellow,
            name=f"shoe_{index}_rear",
        )
        carriage.visual(
            Box((0.05, 0.29, 0.52)),
            origin=Origin(xyz=(x - 0.11, 0.0, 0.0)),
            material=safety_yellow,
            name=f"shoe_{index}_inner_cheek",
        )
        carriage.visual(
            Box((0.05, 0.29, 0.52)),
            origin=Origin(xyz=(x + 0.11, 0.0, 0.0)),
            material=safety_yellow,
            name=f"shoe_{index}_outer_cheek",
        )

        # Bronze wear liners are captured inside each guide shoe but stop just
        # short of the fixed column surfaces.
        carriage.visual(
            Box((0.012, 0.18, 0.40)),
            origin=Origin(xyz=(x - 0.076, 0.0, 0.0)),
            material=bronze,
            name=f"shoe_{index}_left_liner",
        )
        carriage.visual(
            Box((0.012, 0.18, 0.40)),
            origin=Origin(xyz=(x + 0.076, 0.0, 0.0)),
            material=bronze,
            name=f"shoe_{index}_right_liner",
        )
        carriage.visual(
            Box((0.18, 0.012, 0.40)),
            origin=Origin(xyz=(x, -0.086, 0.0)),
            material=bronze,
            name=f"shoe_{index}_front_liner",
        )
        carriage.visual(
            Box((0.18, 0.012, 0.40)),
            origin=Origin(xyz=(x, 0.086, 0.0)),
            material=bronze,
            name=f"shoe_{index}_rear_liner",
        )

    # Exposed bolt heads on the broad carriage plate.
    for index, (x, z) in enumerate(
        (
            (-0.50, 0.17),
            (-0.28, 0.17),
            (0.28, 0.17),
            (0.50, 0.17),
            (-0.50, -0.17),
            (-0.28, -0.17),
            (0.28, -0.17),
            (0.50, -0.17),
        )
    ):
        carriage.visual(
            Cylinder(radius=0.024, length=0.018),
            origin=Origin(xyz=(x, -0.249, z), rpy=(math.pi / 2.0, 0.0, 0.0)),
            material=black_steel,
            name=f"face_bolt_{index}",
        )

    model.articulation(
        "frame_to_carriage",
        ArticulationType.PRISMATIC,
        parent=frame,
        child=carriage,
        origin=Origin(xyz=(0.0, 0.0, 0.75)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=90000.0, velocity=0.18, lower=0.0, upper=0.75),
    )
    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    frame = object_model.get_part("frame")
    carriage = object_model.get_part("carriage")
    slide = object_model.get_articulation("frame_to_carriage")

    ctx.expect_overlap(
        carriage,
        frame,
        axes="z",
        elem_a="shoe_0_front",
        elem_b="guide_column_0",
        min_overlap=0.35,
        name="lower carriage shoe overlaps first column height",
    )
    ctx.expect_overlap(
        carriage,
        frame,
        axes="z",
        elem_a="shoe_1_front",
        elem_b="guide_column_1",
        min_overlap=0.35,
        name="lower carriage shoe overlaps second column height",
    )
    ctx.expect_gap(
        frame,
        carriage,
        axis="z",
        positive_elem="upper_tie_beam",
        negative_elem="shoe_0_front",
        min_gap=0.70,
        name="upper tie beam clears lowered carriage",
    )

    rest_position = ctx.part_world_position(carriage)
    with ctx.pose({slide: 0.75}):
        ctx.expect_overlap(
            carriage,
            frame,
            axes="z",
            elem_a="shoe_0_front",
            elem_b="guide_column_0",
            min_overlap=0.35,
            name="raised shoe remains captured on first column",
        )
        ctx.expect_overlap(
            carriage,
            frame,
            axes="z",
            elem_a="shoe_1_front",
            elem_b="guide_column_1",
            min_overlap=0.35,
            name="raised shoe remains captured on second column",
        )
        ctx.expect_gap(
            frame,
            carriage,
            axis="z",
            positive_elem="upper_tie_beam",
            negative_elem="shoe_0_front",
            min_gap=0.20,
            name="raised carriage stops below upper tie beam",
        )
        raised_position = ctx.part_world_position(carriage)

    ctx.check(
        "carriage travels upward on column axis",
        rest_position is not None
        and raised_position is not None
        and raised_position[2] > rest_position[2] + 0.70,
        details=f"rest={rest_position}, raised={raised_position}",
    )

    return ctx.report()


object_model = build_object_model()
