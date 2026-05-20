from __future__ import annotations

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
    model = ArticulatedObject(name="bridge_tied_dual_column_lift")

    dark_steel = model.material("dark_steel", color=(0.12, 0.13, 0.14, 1.0))
    bridge_grey = model.material("bridge_grey", color=(0.32, 0.35, 0.36, 1.0))
    guide_metal = model.material("guide_metal", color=(0.72, 0.74, 0.72, 1.0))
    carriage_yellow = model.material("carriage_yellow", color=(0.95, 0.64, 0.10, 1.0))
    sleeve_grey = model.material("sleeve_grey", color=(0.23, 0.25, 0.26, 1.0))
    black = model.material("black", color=(0.02, 0.02, 0.018, 1.0))

    fixed = model.part("fixed_frame")

    # Fixed, grounded bridge frame: two columns physically tied by a lower foot
    # and a top bridge.  The columns start on the lower foot and touch the upper
    # bridge so the fixed part reads as one welded frame.
    for i, x in enumerate((-0.42, 0.42)):
        fixed.visual(
            Box((0.10, 0.10, 1.85)),
            origin=Origin(xyz=(x, 0.0, 1.045)),
            material=dark_steel,
            name=f"column_{i}",
        )
        fixed.visual(
            Box((0.28, 0.72, 0.05)),
            origin=Origin(xyz=(x, 0.0, 0.025)),
            material=dark_steel,
            name=f"base_plate_{i}",
        )
        fixed.visual(
            Box((0.032, 0.020, 1.58)),
            origin=Origin(xyz=(x, -0.060, 1.025)),
            material=guide_metal,
            name=f"guide_strip_{i}",
        )

    fixed.visual(
        Box((1.08, 0.18, 0.12)),
        origin=Origin(xyz=(0.0, 0.0, 0.060)),
        material=bridge_grey,
        name="lower_foot",
    )
    fixed.visual(
        Box((1.08, 0.16, 0.14)),
        origin=Origin(xyz=(0.0, 0.0, 2.040)),
        material=bridge_grey,
        name="upper_bridge",
    )

    for i, x in enumerate((-0.42, 0.42)):
        fixed.visual(
            Box((0.24, 0.08, 0.10)),
            origin=Origin(xyz=(x, -0.085, 0.170)),
            material=bridge_grey,
            name=f"lower_gusset_{i}",
        )
        fixed.visual(
            Box((0.24, 0.07, 0.10)),
            origin=Origin(xyz=(x, -0.075, 1.905)),
            material=bridge_grey,
            name=f"upper_gusset_{i}",
        )

    for i, (x, y) in enumerate(
        (
            (-0.50, -0.25),
            (-0.34, 0.25),
            (0.34, -0.25),
            (0.50, 0.25),
        )
    ):
        fixed.visual(
            Cylinder(radius=0.025, length=0.018),
            origin=Origin(xyz=(x, y, 0.055)),
            material=black,
            name=f"anchor_bolt_{i}",
        )

    carriage = model.part("front_carriage")

    # The moving carriage frame is built in its own joint frame, centered at the
    # middle of the guide sleeves.  It sits on the front side of the columns and
    # surrounds each column with clearance rather than intersecting it.
    sleeve_inner_half = 0.075
    sleeve_wall = 0.025
    sleeve_outer_half = sleeve_inner_half + sleeve_wall
    sleeve_height = 0.42
    for i, x in enumerate((-0.42, 0.42)):
        carriage.visual(
            Box((2.0 * sleeve_outer_half, sleeve_wall, sleeve_height)),
            origin=Origin(xyz=(x, -sleeve_inner_half - sleeve_wall / 2.0, 0.0)),
            material=sleeve_grey,
            name=f"sleeve_{i}_front",
        )
        carriage.visual(
            Box((2.0 * sleeve_outer_half, sleeve_wall, sleeve_height)),
            origin=Origin(xyz=(x, sleeve_inner_half + sleeve_wall / 2.0, 0.0)),
            material=sleeve_grey,
            name=f"sleeve_{i}_back",
        )
        carriage.visual(
            Box((sleeve_wall, 2.0 * sleeve_inner_half, sleeve_height)),
            origin=Origin(xyz=(x - sleeve_inner_half - sleeve_wall / 2.0, 0.0, 0.0)),
            material=sleeve_grey,
            name=f"sleeve_{i}_outer",
        )
        carriage.visual(
            Box((sleeve_wall, 2.0 * sleeve_inner_half, sleeve_height)),
            origin=Origin(xyz=(x + sleeve_inner_half + sleeve_wall / 2.0, 0.0, 0.0)),
            material=sleeve_grey,
            name=f"sleeve_{i}_inner",
        )
        carriage.visual(
            Box((0.070, 0.005, 0.18)),
            origin=Origin(xyz=(x, -0.0725, 0.0)),
            material=guide_metal,
            name=f"guide_shoe_{i}",
        )

    carriage.visual(
        Box((0.92, 0.070, 0.10)),
        origin=Origin(xyz=(0.0, -0.130, 0.135)),
        material=carriage_yellow,
        name="upper_crossbeam",
    )
    carriage.visual(
        Box((0.92, 0.070, 0.10)),
        origin=Origin(xyz=(0.0, -0.130, -0.135)),
        material=carriage_yellow,
        name="lower_crossbeam",
    )
    carriage.visual(
        Box((0.56, 0.035, 0.34)),
        origin=Origin(xyz=(0.0, -0.170, 0.0)),
        material=carriage_yellow,
        name="front_plate",
    )
    for i, x in enumerate((-0.28, 0.28)):
        carriage.visual(
            Box((0.055, 0.085, 0.34)),
            origin=Origin(xyz=(x, -0.132, 0.0)),
            material=carriage_yellow,
            name=f"web_plate_{i}",
        )

    model.articulation(
        "carriage_lift",
        ArticulationType.PRISMATIC,
        parent=fixed,
        child=carriage,
        origin=Origin(xyz=(0.0, 0.0, 0.55)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=1500.0, velocity=0.35, lower=0.0, upper=1.05),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)
    fixed = object_model.get_part("fixed_frame")
    carriage = object_model.get_part("front_carriage")
    lift = object_model.get_articulation("carriage_lift")

    ctx.check(
        "one vertical prismatic carriage",
        lift.articulation_type == ArticulationType.PRISMATIC
        and lift.axis == (0.0, 0.0, 1.0)
        and lift.motion_limits is not None
        and lift.motion_limits.lower == 0.0
        and lift.motion_limits.upper >= 1.0,
        details=f"type={lift.articulation_type}, axis={lift.axis}, limits={lift.motion_limits}",
    )

    for i in (0, 1):
        ctx.expect_gap(
            fixed,
            carriage,
            axis="y",
            min_gap=0.018,
            max_gap=0.035,
            positive_elem=f"column_{i}",
            negative_elem=f"sleeve_{i}_front",
            name=f"sleeve_{i} front clearance",
        )
        ctx.expect_gap(
            carriage,
            fixed,
            axis="y",
            min_gap=0.018,
            max_gap=0.035,
            positive_elem=f"sleeve_{i}_back",
            negative_elem=f"column_{i}",
            name=f"sleeve_{i} back clearance",
        )
        ctx.expect_overlap(
            carriage,
            fixed,
            axes="z",
            min_overlap=0.35,
            elem_a=f"sleeve_{i}_front",
            elem_b=f"column_{i}",
            name=f"sleeve_{i} engages column",
        )
        ctx.expect_contact(
            carriage,
            fixed,
            elem_a=f"guide_shoe_{i}",
            elem_b=f"guide_strip_{i}",
            name=f"guide_shoe_{i} bears on guide strip",
        )

    ctx.expect_gap(
        fixed,
        carriage,
        axis="y",
        min_gap=0.035,
        positive_elem="column_0",
        negative_elem="front_plate",
        name="carriage is in front of columns",
    )

    rest_pos = ctx.part_world_position(carriage)
    with ctx.pose({lift: lift.motion_limits.upper}):
        raised_pos = ctx.part_world_position(carriage)
        ctx.expect_gap(
            fixed,
            carriage,
            axis="z",
            min_gap=0.12,
            positive_elem="upper_bridge",
            negative_elem="sleeve_0_front",
            name="raised carriage clears upper bridge",
        )
        ctx.expect_overlap(
            carriage,
            fixed,
            axes="z",
            min_overlap=0.35,
            elem_a="sleeve_0_front",
            elem_b="column_0",
            name="raised sleeve stays on column",
        )

    ctx.check(
        "carriage moves upward",
        rest_pos is not None
        and raised_pos is not None
        and raised_pos[2] > rest_pos[2] + 1.0,
        details=f"rest={rest_pos}, raised={raised_pos}",
    )

    return ctx.report()


object_model = build_object_model()
