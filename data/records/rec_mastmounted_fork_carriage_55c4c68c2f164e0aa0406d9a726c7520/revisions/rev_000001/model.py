from __future__ import annotations

from sdk import (
    ArticulatedObject,
    ArticulationType,
    Box,
    Material,
    MotionLimits,
    Origin,
    TestContext,
    TestReport,
)


def build_object_model() -> ArticulatedObject:
    model = ArticulatedObject(name="compact_lift_mast_fork_module")

    mast_steel = Material("dark_powder_coated_steel", rgba=(0.08, 0.09, 0.10, 1.0))
    safety_yellow = Material("safety_yellow_carriage", rgba=(0.95, 0.67, 0.08, 1.0))
    wear_black = Material("black_polymer_wear_pads", rgba=(0.015, 0.015, 0.012, 1.0))
    fork_steel = Material("brushed_fork_steel", rgba=(0.55, 0.57, 0.56, 1.0))

    mast = model.part("mast")
    mast.visual(
        Box((0.74, 0.38, 0.06)),
        origin=Origin(xyz=(0.0, 0.0, 0.03)),
        material=mast_steel,
        name="base_plate",
    )
    mast.visual(
        Box((0.065, 0.085, 1.18)),
        origin=Origin(xyz=(-0.22, 0.0, 0.65)),
        material=mast_steel,
        name="column_0",
    )
    mast.visual(
        Box((0.065, 0.085, 1.18)),
        origin=Origin(xyz=(0.22, 0.0, 0.65)),
        material=mast_steel,
        name="column_1",
    )
    mast.visual(
        Box((0.58, 0.13, 0.08)),
        origin=Origin(xyz=(0.0, 0.0, 1.28)),
        material=mast_steel,
        name="crosshead",
    )
    mast.visual(
        Box((0.56, 0.10, 0.07)),
        origin=Origin(xyz=(0.0, -0.02, 0.13)),
        material=mast_steel,
        name="lower_tie",
    )

    carriage = model.part("carriage")
    carriage.visual(
        Box((0.66, 0.040, 0.36)),
        origin=Origin(xyz=(0.0, 0.115, 0.0)),
        material=safety_yellow,
        name="carriage_plate",
    )

    carriage.visual(
        Box((0.18, 0.035, 0.30)),
        origin=Origin(xyz=(-0.22, 0.090, 0.0)),
        material=safety_yellow,
        name="guide_front_0",
    )
    carriage.visual(
        Box((0.18, 0.035, 0.30)),
        origin=Origin(xyz=(-0.22, -0.090, 0.0)),
        material=safety_yellow,
        name="guide_rear_0",
    )
    carriage.visual(
        Box((0.040, 0.215, 0.30)),
        origin=Origin(xyz=(-0.315, 0.0, 0.0)),
        material=safety_yellow,
        name="guide_outer_0",
    )
    carriage.visual(
        Box((0.040, 0.215, 0.30)),
        origin=Origin(xyz=(-0.125, 0.0, 0.0)),
        material=safety_yellow,
        name="guide_inner_0",
    )
    carriage.visual(
        Box((0.050, 0.030, 0.24)),
        origin=Origin(xyz=(-0.22, 0.0575, 0.0)),
        material=wear_black,
        name="front_wear_pad_0",
    )
    carriage.visual(
        Box((0.050, 0.030, 0.24)),
        origin=Origin(xyz=(-0.22, -0.0575, 0.0)),
        material=wear_black,
        name="rear_wear_pad_0",
    )
    carriage.visual(
        Box((0.18, 0.035, 0.30)),
        origin=Origin(xyz=(0.22, 0.090, 0.0)),
        material=safety_yellow,
        name="guide_front_1",
    )
    carriage.visual(
        Box((0.18, 0.035, 0.30)),
        origin=Origin(xyz=(0.22, -0.090, 0.0)),
        material=safety_yellow,
        name="guide_rear_1",
    )
    carriage.visual(
        Box((0.040, 0.215, 0.30)),
        origin=Origin(xyz=(0.315, 0.0, 0.0)),
        material=safety_yellow,
        name="guide_outer_1",
    )
    carriage.visual(
        Box((0.040, 0.215, 0.30)),
        origin=Origin(xyz=(0.125, 0.0, 0.0)),
        material=safety_yellow,
        name="guide_inner_1",
    )
    carriage.visual(
        Box((0.050, 0.030, 0.24)),
        origin=Origin(xyz=(0.22, 0.0575, 0.0)),
        material=wear_black,
        name="front_wear_pad_1",
    )
    carriage.visual(
        Box((0.050, 0.030, 0.24)),
        origin=Origin(xyz=(0.22, -0.0575, 0.0)),
        material=wear_black,
        name="rear_wear_pad_1",
    )

    for index, x in enumerate((-0.12, 0.12)):
        carriage.visual(
            Box((0.085, 0.110, 0.32)),
            origin=Origin(xyz=(x, 0.185, -0.02)),
            material=fork_steel,
            name=f"fork_heel_{index}",
        )
        carriage.visual(
            Box((0.070, 0.66, 0.055)),
            origin=Origin(xyz=(x, 0.545, -0.205)),
            material=fork_steel,
            name=f"fork_tine_{index}",
        )

    model.articulation(
        "mast_to_carriage",
        ArticulationType.PRISMATIC,
        parent=mast,
        child=carriage,
        origin=Origin(xyz=(0.0, 0.0, 0.42)),
        axis=(0.0, 0.0, 1.0),
        motion_limits=MotionLimits(effort=850.0, velocity=0.35, lower=0.0, upper=0.55),
    )

    return model


def run_tests() -> TestReport:
    ctx = TestContext(object_model)

    mast = object_model.get_part("mast")
    carriage = object_model.get_part("carriage")
    lift = object_model.get_articulation("mast_to_carriage")
    limits = lift.motion_limits

    ctx.check(
        "single vertical prismatic lift",
        len(object_model.articulations) == 1
        and lift.articulation_type == ArticulationType.PRISMATIC
        and tuple(lift.axis) == (0.0, 0.0, 1.0)
        and limits is not None
        and limits.lower == 0.0
        and limits.upper is not None
        and limits.upper > 0.50,
        details=f"joint={lift}, limits={limits}",
    )

    for index in (0, 1):
        ctx.expect_gap(
            carriage,
            mast,
            axis="y",
            positive_elem=f"guide_front_{index}",
            negative_elem=f"column_{index}",
            min_gap=0.025,
            max_gap=0.040,
            name=f"front guide clearance {index}",
        )
        ctx.expect_gap(
            mast,
            carriage,
            axis="y",
            positive_elem=f"column_{index}",
            negative_elem=f"guide_rear_{index}",
            min_gap=0.025,
            max_gap=0.040,
            name=f"rear guide clearance {index}",
        )
        ctx.expect_contact(
            carriage,
            mast,
            elem_a=f"front_wear_pad_{index}",
            elem_b=f"column_{index}",
            contact_tol=0.001,
            name=f"front wear pad touches column {index}",
        )
        ctx.expect_contact(
            carriage,
            mast,
            elem_a=f"rear_wear_pad_{index}",
            elem_b=f"column_{index}",
            contact_tol=0.001,
            name=f"rear wear pad touches column {index}",
        )
        ctx.expect_overlap(
            carriage,
            mast,
            axes="z",
            elem_a=f"guide_front_{index}",
            elem_b=f"column_{index}",
            min_overlap=0.28,
            name=f"guide wraps column height {index}",
        )

    ctx.expect_gap(
        mast,
        carriage,
        axis="x",
        positive_elem="column_0",
        negative_elem="guide_outer_0",
        min_gap=0.035,
        max_gap=0.055,
        name="outer guide clearance 0",
    )
    ctx.expect_gap(
        carriage,
        mast,
        axis="x",
        positive_elem="guide_inner_0",
        negative_elem="column_0",
        min_gap=0.035,
        max_gap=0.055,
        name="inner guide clearance 0",
    )
    ctx.expect_gap(
        mast,
        carriage,
        axis="x",
        positive_elem="column_1",
        negative_elem="guide_inner_1",
        min_gap=0.035,
        max_gap=0.055,
        name="inner guide clearance 1",
    )
    ctx.expect_gap(
        carriage,
        mast,
        axis="x",
        positive_elem="guide_outer_1",
        negative_elem="column_1",
        min_gap=0.035,
        max_gap=0.055,
        name="outer guide clearance 1",
    )

    rest_position = ctx.part_world_position(carriage)
    with ctx.pose({lift: limits.upper if limits and limits.upper is not None else 0.0}):
        raised_position = ctx.part_world_position(carriage)
        ctx.expect_gap(
            mast,
            carriage,
            axis="z",
            positive_elem="crosshead",
            negative_elem="carriage_plate",
            min_gap=0.05,
            name="raised carriage clears crosshead",
        )
        ctx.expect_overlap(
            carriage,
            mast,
            axes="z",
            elem_a="guide_front_0",
            elem_b="column_0",
            min_overlap=0.28,
            name="raised guide remains on mast",
        )

    ctx.check(
        "carriage raises along mast",
        rest_position is not None
        and raised_position is not None
        and raised_position[2] > rest_position[2] + 0.50,
        details=f"rest={rest_position}, raised={raised_position}",
    )

    return ctx.report()


object_model = build_object_model()
